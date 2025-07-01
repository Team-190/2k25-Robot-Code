package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureEdges.AlgaeEdge;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureEdges.EdgeCommand;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
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
 * The V2_RedundancySuperstructure class manages the coordinated movement and state transitions of
 * the robot's major subsystems including elevator, funnel, manipulator, and intake.
 */
public class V2_RedundancySuperstructure extends SubsystemBase {

  private final Graph<V2_RedundancySuperstructureStates, EdgeCommand> graph;
  private final V2_RedundancyElevator elevator;
  private final V2_RedundancyFunnel funnel;
  private final V2_RedundancyManipulator manipulator;
  private final V2_RedundancyIntake intake;

  /**
   * The previous, current, and next states of the superstructure. These are used to track the state
   * transitions and manage the command scheduling.
   */
  @Getter private V2_RedundancySuperstructureStates previousState;

  /**
   * The current state of the superstructure, which is updated periodically based on the command
   * scheduling and state transitions.
   */
  @Getter private V2_RedundancySuperstructureStates currentState;

  /**
   * The next state that the superstructure is transitioning to. This is determined by the command
   * scheduling and the current target state.
   */
  @Getter private V2_RedundancySuperstructureStates nextState;

  /**
   * The target state that the superstructure is trying to achieve. This is set by the robot and
   * determines the next action to be taken.
   */
  @Getter private V2_RedundancySuperstructureStates targetState;

  /** The command that is currently being executed to transition between states. */
  private EdgeCommand edgeCommand;

  public V2_RedundancySuperstructure(
      V2_RedundancyElevator elevator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake) {
    this.elevator = elevator;
    this.funnel = funnel;
    this.manipulator = manipulator;
    this.intake = intake;

    previousState = null;
    currentState = V2_RedundancySuperstructureStates.START;
    nextState = null;

    targetState = V2_RedundancySuperstructureStates.START;

    // Initialize the graph
    graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    for (V2_RedundancySuperstructureStates vertex : V2_RedundancySuperstructureStates.values()) {
      graph.addVertex(vertex);
    }

    // Add edges between states
    V2_RedundancySuperstructureEdges.addEdges(graph, elevator, manipulator, funnel, intake);
  }

  /**
   * Periodic method that handles state transitions and subsystem updates. Updates robot state
   * variables and manages command scheduling based on current state.
   */
  @Override
  public void periodic() {
    elevator.periodic();
    funnel.periodic();
    manipulator.periodic();
    intake.periodic();

    // Set RobotState variables
    RobotState.setIntakingCoral(targetState == V2_RedundancySuperstructureStates.INTAKE_STATION);
    funnel.setManipulatorHasCoral(manipulator.hasCoral());

    if (DriverStation.isDisabled()) {
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
      } else {
        // Run action if we are already at the goal
        if (targetState != V2_RedundancySuperstructureStates.OVERRIDE) {
          targetState.getAction().get(manipulator, funnel, intake);
        }
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
      Logger.recordOutput(NTPrefixes.SUPERSTRUCTURE + "EdgeCommand", "");
    }
  }

  /**
   * Updates the target state and handles command rescheduling for optimal path.
   *
   * @param goal New target state to achieve
   */
  private void setGoal(V2_RedundancySuperstructureStates goal) {
    // Don't do anything if goal is the same
    if (this.targetState == goal) return;
    else {
      this.targetState = goal;
      stopActions();
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
  private Optional<V2_RedundancySuperstructureStates> bfs(
      V2_RedundancySuperstructureStates start, V2_RedundancySuperstructureStates goal) {
    Map<V2_RedundancySuperstructureStates, V2_RedundancySuperstructureStates> parents =
        new HashMap<>();
    Queue<V2_RedundancySuperstructureStates> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null);
    while (!queue.isEmpty()) {
      V2_RedundancySuperstructureStates current = queue.poll();
      if (current == goal) break;
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        V2_RedundancySuperstructureStates neighbor = graph.getEdgeTarget(edge);
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    if (!parents.containsKey(goal)) return Optional.empty();

    V2_RedundancySuperstructureStates nextState = goal;
    while (!nextState.equals(start)) {
      V2_RedundancySuperstructureStates parent = parents.get(nextState);
      if (parent == null) return Optional.empty();
      else if (parent.equals(start)) return Optional.of(nextState);
      nextState = parent;
    }
    return Optional.of(nextState);
  }

  /** Stops all roller actions across funnel, manipulator, and intake subsystems. */
  private void stopActions() {
    funnel.setRollerGoal(FunnelRollerState.STOP);
    manipulator.runManipulator(ManipulatorRollerState.STOP);
    intake.setRollerGoal(IntakeRollerState.STOP);
  }

  /**
   * Checks if a state transition is allowed based on algae presence.
   *
   * @param edge The transition edge to check
   * @param goal The target state
   * @return true if the transition is allowed
   */
  private boolean isEdgeAllowed(EdgeCommand edge, V2_RedundancySuperstructureStates goal) {
    return edge.getAlgaeEdgeType() == AlgaeEdge.NONE
        || RobotState.isHasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE);
  }

  /** Resets the superstructure to initial auto state. */
  public void setAutoStart() {
    currentState = V2_RedundancySuperstructureStates.START;
    nextState = null;
    targetState = V2_RedundancySuperstructureStates.STOW_DOWN;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
  }

  /**
   * Maps current OI reef height to corresponding elevator position state.
   *
   * @return Appropriate superstructure state for current reef height
   */
  private V2_RedundancySuperstructureStates getElevatorPosition() {
    switch (RobotState.getOIData().currentReefHeight()) {
      case STOW, CORAL_INTAKE -> {
        return V2_RedundancySuperstructureStates.STOW_DOWN;
      }
      case L1 -> {
        return V2_RedundancySuperstructureStates.L1;
      }
      case L2 -> {
        return V2_RedundancySuperstructureStates.L2;
      }
      case L3 -> {
        return V2_RedundancySuperstructureStates.L3;
      }
      case L4 -> {
        return V2_RedundancySuperstructureStates.L4;
      }
      default -> {
        return V2_RedundancySuperstructureStates.START;
      }
    }
  }

  /** Control Commands * */
  public Command runGoal(V2_RedundancySuperstructureStates goal) {
    return runOnce(() -> setGoal(goal));
  }

  public Command runGoal(Supplier<V2_RedundancySuperstructureStates> goal) {
    return runOnce(() -> setGoal(goal.get()));
  }

  @AutoLogOutput(key = NTPrefixes.SUPERSTRUCTURE + "At Goal")
  public boolean atGoal() {
    return currentState == targetState;
  }

  public Command override(Runnable action, V2_RedundancySuperstructureStates oldGoal) {
    return Commands.sequence(
            runGoal(V2_RedundancySuperstructureStates.OVERRIDE), Commands.run(action))
        .finallyDo(() -> setGoal(oldGoal));
  }

  public Command runGoalUntil(V2_RedundancySuperstructureStates goal, BooleanSupplier condition) {
    return Commands.sequence(runGoal(goal), Commands.waitUntil(condition));
  }

  public Command runPreviousState() {
    return runGoal(() -> previousState).withTimeout(0.02);
  }

  public Command runReefGoal(Supplier<ReefState> goal) {
    return runGoal(
        () -> {
          switch (goal.get()) {
            case L1:
              return V2_RedundancySuperstructureStates.L1;
            case L2:
              return V2_RedundancySuperstructureStates.L2;
            case L3:
              return V2_RedundancySuperstructureStates.L3;
            case L4:
              return V2_RedundancySuperstructureStates.L4;
            case L4_PLUS:
              return V2_RedundancySuperstructureStates.L4_PLUS;
            default:
              return V2_RedundancySuperstructureStates.STOW_DOWN;
          }
        });
  }

  public Command runReefScoreGoal(Supplier<ReefState> goal) {
    switch (goal.get()) {
      case L1:
        return runActionWithTimeout(
            V2_RedundancySuperstructureStates.L1, V2_RedundancySuperstructureStates.SCORE_L1, 0.8);
      case L2:
        return runActionWithTimeout(
            V2_RedundancySuperstructureStates.L2, V2_RedundancySuperstructureStates.SCORE_L2, 0.15);
      case L3:
        return runActionWithTimeout(
            V2_RedundancySuperstructureStates.L3, V2_RedundancySuperstructureStates.SCORE_L3, 0.15);
      case L4:
        return runActionWithTimeout(
            V2_RedundancySuperstructureStates.L4, V2_RedundancySuperstructureStates.SCORE_L4, 0.4);
      case L4_PLUS:
        return runActionWithTimeout(
            V2_RedundancySuperstructureStates.L4_PLUS,
            V2_RedundancySuperstructureStates.SCORE_L4_PLUS,
            0.5);
      default:
        return Commands.none();
    }
  }

  public Command runActionWithTimeout(
      V2_RedundancySuperstructureStates pose,
      V2_RedundancySuperstructureStates action,
      double timeout) {
    return Commands.sequence(
        runGoal(pose),
        Commands.waitUntil(() -> atGoal()),
        runGoal(action),
        Commands.waitSeconds(timeout),
        runGoal(pose));
  }

  public Command setPosition() {
    return runGoal(() -> getElevatorPosition()).withTimeout(0.02);
  }
}
