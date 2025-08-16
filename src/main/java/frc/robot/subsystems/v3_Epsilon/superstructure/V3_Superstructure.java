package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotMode;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructureEdges.EdgeCommand;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructureEdges.GamePieceEdge;
import frc.robot.util.NTPrefixes;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The V3_Superstructure class manages the coordinated movement and state transitions of the robot's
 * major subsystems including elevator, funnel, manipulator, and intake.
 */
public class V3_Superstructure extends SubsystemBase {

  private final Graph<V3_SuperstructureStates, EdgeCommand> graph;
  private final ElevatorFSM elevator;
  private final V3_EpsilonIntake intake;
  private final V3_EpsilonManipulator manipulator;

  /**
   * The previous, current, and next states of the superstructure. These are used to track the state
   * transitions and manage the command scheduling.
   */
  @Getter private V3_SuperstructureStates previousState;

  /**
   * The current state of the superstructure, which is updated periodically based on the command
   * scheduling and state transitions.
   */
  @Getter private V3_SuperstructureStates currentState;

  /**
   * The next state that the superstructure is transitioning to. This is determined by the command
   * scheduling and the current target state.
   */
  @Getter private V3_SuperstructureStates nextState;

  /**
   * The target state that the superstructure is trying to achieve. This is set by the robot and
   * determines the next action to be taken.
   */
  @Getter private V3_SuperstructureStates targetState;

  /** The command that is currently being executed to transition between states. */
  private EdgeCommand edgeCommand;

  /**
   * Constructs a V3_Superstructure.
   *
   * @param elevator The elevator subsystem.
   * @param funnel The funnel subsystem.
   * @param intake The intake subsystem.
   * @param manipulator The manipulator subsystem.
   */
  public V3_Superstructure(
      ElevatorFSM elevator, V3_EpsilonIntake intake, V3_EpsilonManipulator manipulator) {
    this.elevator = elevator;
    this.intake = intake;
    this.manipulator = manipulator;

    previousState = null;
    currentState = V3_SuperstructureStates.START;
    nextState = null;

    targetState = V3_SuperstructureStates.START;

    // Initialize the graph
    graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    for (V3_SuperstructureStates vertex : V3_SuperstructureStates.values()) {
      graph.addVertex(vertex);
    }
  }

  /**
   * Periodic method that handles state transitions and subsystem updates. Updates robot state
   * variables and manages command scheduling based on current state.
   */
  @Override
  public void periodic() {

    if (RobotMode.disabled()) {
      nextState = null;
    } else if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
      // Update edge to new state
      if (nextState != null) {
        previousState = currentState;
        currentState = nextState;
        nextState = null;
      }

      // Schedule next command in sequence
      if (currentState != targetState) {
        bfs(currentState, targetState)
            .ifPresent(
                next -> {
                  this.nextState = next;
                  edgeCommand = graph.getEdge(currentState, next);
                  edgeCommand.getCommand().schedule();
                });
      }
    }

    // Log the current state of the superstructure and edge command
    Logger.recordOutput(
        NTPrefixes.SUPERSTRUCTURE + "Goal", targetState == null ? "NULL" : targetState.toString());
    Logger.recordOutput(
        NTPrefixes.SUPERSTRUCTURE + "Previous State",
        previousState == null ? "NULL" : previousState.toString());
    Logger.recordOutput(NTPrefixes.SUPERSTRUCTURE + "Current State", currentState.toString());
    Logger.recordOutput(
        NTPrefixes.SUPERSTRUCTURE + "Next State",
        nextState == null ? "NULL" : nextState.toString());
    if (edgeCommand != null) {
      Logger.recordOutput(
          NTPrefixes.SUPERSTRUCTURE + "EdgeCommand",
          graph.getEdgeSource(edgeCommand) + " --> " + graph.getEdgeTarget(edgeCommand));
    } else {
      Logger.recordOutput(NTPrefixes.SUPERSTRUCTURE + "EdgeCommand", "NO EDGES SCHEDULED");
    }

    elevator.periodic();
    intake.periodic();
    manipulator.periodic();
  }

  /**
   * Updates the target state and handles command rescheduling for optimal path.
   *
   * @param goal New target state to achieve
   */
  private void setGoal(V3_SuperstructureStates goal) {
    // Don't do anything if goal is the same
    if (this.targetState == goal) return;
    else {
      this.targetState = goal;
    }

    if (nextState == null) return;

    var edgeToCurrentState = graph.getEdge(nextState, currentState);
    // Figure out if we should schedule a different command to get to goal faster
    if (edgeCommand.getCommand().isScheduled()
        && edgeToCurrentState != null
        && isEdgeAllowed(edgeToCurrentState, goal)) {
      // Figure out where we would have gone from the previous state
      bfs(currentState, goal)
          .ifPresent(
              newNext -> {
                if (newNext == nextState) {
                  // We are already on track
                  return;
                }

                if (newNext != currentState && graph.getEdge(nextState, newNext) != null) {
                  // We can skip directly to the newNext edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(currentState, newNext);
                  edgeCommand.getCommand().schedule();
                  nextState = newNext;
                } else {
                  // Follow the reverse edge from next back to the current edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(nextState, currentState);
                  edgeCommand.getCommand().schedule();
                  var temp = currentState;
                  previousState = currentState;
                  currentState = nextState;
                  nextState = temp;
                }
              });
    }
  }

  /**
   * Performs breadth-first search to find the next state in the path to the goal.
   *
   * @param start Starting state
   * @param goal Target state
   * @return Optional containing the next state in the path, empty if no path exists
   */
  private Optional<V3_SuperstructureStates> bfs(
      V3_SuperstructureStates start, V3_SuperstructureStates goal) {
    Map<V3_SuperstructureStates, V3_SuperstructureStates> parents = new HashMap<>();
    Queue<V3_SuperstructureStates> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null);
    while (!queue.isEmpty()) {
      V3_SuperstructureStates current = queue.poll();
      if (current == goal) break;
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        V3_SuperstructureStates neighbor = graph.getEdgeTarget(edge);
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    if (!parents.containsKey(goal)) return Optional.empty();

    V3_SuperstructureStates nextState = goal;
    while (!nextState.equals(start)) {
      V3_SuperstructureStates parent = parents.get(nextState);
      if (parent == null) return Optional.empty();
      else if (parent.equals(start)) return Optional.of(nextState);
      nextState = parent;
    }
    return Optional.of(nextState);
  }

  /**
   * Checks if a state transition is allowed based on algae presence.
   *
   * @param edge The transition edge to check
   * @param goal The target state
   * @return true if the transition is allowed
   */
  private boolean isEdgeAllowed(EdgeCommand edge, V3_SuperstructureStates goal) { // Change later
    return edge.getEdgeType() == GamePieceEdge.NONE
        || RobotState.isHasAlgae() == (edge.getEdgeType() == GamePieceEdge.ALGAE);
  }

  /** Resets the superstructure to initial auto state. */
  public void setAutoStart() {
    currentState = V3_SuperstructureStates.START;
    nextState = null;
    targetState = V3_SuperstructureStates.STOW_DOWN;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
  }
  // --- Control Commands ---

  /**
   * Returns a command that sets the superstructure to the given goal state.
   *
   * @param goal The desired superstructure state
   * @return Command to run the goal
   */
  public Command runGoal(V3_SuperstructureStates goal) {
    return runOnce(() -> setGoal(goal));
  }

  /**
   * Returns a command that sets the superstructure to the goal provided by a supplier.
   *
   * @param goal Supplier providing the desired superstructure state
   * @return Command to run the goal
   */
  public Command runGoal(Supplier<V3_SuperstructureStates> goal) {
    return runOnce(() -> setGoal(goal.get()));
  }

  /**
   * Checks whether the superstructure has reached its target state.
   *
   * @return true if current state matches the target state
   */
  @AutoLogOutput(key = NTPrefixes.SUPERSTRUCTURE + "At Goal")
  public boolean atGoal() {
    return currentState == targetState;
  }

  /**
   * Runs a temporary override action, returning to a previous goal after.
   *
   * @param action The override action to perform
   * @param oldGoal The goal to return to after override
   * @return Command that runs the override and resumes the old goal
   */
  public Command override(Runnable action) {
    return Commands.sequence(runGoal(V3_SuperstructureStates.OVERRIDE), Commands.run(action))
        .finallyDo(() -> setGoal(currentState));
  }

  /**
   * Runs a temporary override action, returning to a previous goal after.
   *
   * @param action The override action to perform
   * @param oldGoal The goal to return to after override
   * @return Command that runs the override and resumes the old goal
   */
  public Command override(Runnable action, double timeSeconds) {
    return override(action).withTimeout(timeSeconds);
  }

  /**
   * Runs the goal state and waits until a given condition becomes true.
   *
   * @param goal The desired superstructure state
   * @param condition The condition to wait for after running the goal
   * @return Combined command for running and waiting
   */
  public Command runGoalUntil(V3_SuperstructureStates goal, BooleanSupplier condition) {
    return Commands.sequence(runGoal(goal), Commands.waitUntil(condition));
  }

  public Command runGoalUntil(Supplier<V3_SuperstructureStates> goal, BooleanSupplier condition) {
    return Commands.sequence(runGoal(goal), Commands.waitUntil(condition));
  }

  /**
   * Returns a short command to run the previous state, useful for temporary state restoration.
   *
   * @return Command to go back to the previous state
   */
  public Command runPreviousState() {
    return runGoal(() -> previousState);
  }

  /**
   * Runs an action by going to a pose, performing the action, waiting, and returning.
   *
   * @param pose The pose to return to after action
   * @param action The action to perform
   * @param timeout How long to wait during the action phase (in seconds)
   * @return Full command sequence
   */
  public Command runActionWithTimeout(
      V3_SuperstructureStates pose, V3_SuperstructureStates action, double timeout) {
    return Commands.sequence(
            runGoal(action), // Run the action
            Commands.waitUntil(() -> atGoal()),
            Commands.waitSeconds(timeout),
            runGoal(pose))
        .finallyDo(() -> setGoal(pose)); // Return to original pose
  }

  /**
   * Checks if the elevator is at its goal position.
   *
   * @return True if the elevator is at its goal, false otherwise.
   */
  public boolean elevatorAtGoal() {
    return elevator.atGoal();
  }
}
