package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotMode;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureEdges.AlgaeEdge;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureEdges.EdgeCommand;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.util.NTPrefixes;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
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

    if (currentState == V2_RedundancySuperstructureStates.L4
        && nextState == V2_RedundancySuperstructureStates.SCORE_L4
        && elevator.atGoal()) {
      targetState.getAction().get(manipulator, funnel, intake);
    } else if (currentState != null
        && !currentState.equals(V2_RedundancySuperstructureStates.OVERRIDE)) {
      currentState.getAction().get(manipulator, funnel, intake);
    }

    // Set RobotState variables
    RobotState.setIntakingCoral(targetState == V2_RedundancySuperstructureStates.INTAKE_STATION);
    if ((RobotState.isIntakingAlgae() || manipulator.isIntakingAlgae())
        && (targetState == V2_RedundancySuperstructureStates.INTAKE_FLOOR
            || targetState == V2_RedundancySuperstructureStates.INTAKE_REEF_L2
            || targetState == V2_RedundancySuperstructureStates.INTAKE_REEF_L3)) {
      RobotState.setIntakingAlgae(true);
    } else {
      RobotState.setIntakingAlgae(false);
    }
    funnel.setManipulatorHasCoral(manipulator.hasCoral());

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
    manipulator.periodic();
    funnel.periodic();
    intake.periodic();
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
      case L4_PLUS -> {
        return V2_RedundancySuperstructureStates.L4_PLUS;
      }
      case ALGAE_FLOOR_INTAKE -> {
        return V2_RedundancySuperstructureStates.FLOOR_ACQUISITION;
      }
      case ALGAE_INTAKE_BOTTOM -> {
        return V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2;
      }
      case ALGAE_INTAKE_TOP -> {
        return V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3;
      }
      case ALGAE_SCORE -> {
        return V2_RedundancySuperstructureStates.BARGE;
      }
      default -> {
        return V2_RedundancySuperstructureStates.START;
      }
    }
  }

  // --- Control Commands ---

  /**
   * Returns a command that sets the superstructure to the given goal state.
   *
   * @param goal The desired superstructure state
   * @return Command to run the goal
   */
  public Command runGoal(V2_RedundancySuperstructureStates goal) {
    return runOnce(() -> setGoal(goal));
  }

  /**
   * Returns a command that sets the superstructure to the goal provided by a supplier.
   *
   * @param goal Supplier providing the desired superstructure state
   * @return Command to run the goal
   */
  public Command runGoal(Supplier<V2_RedundancySuperstructureStates> goal) {
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
    return Commands.sequence(
            runGoal(V2_RedundancySuperstructureStates.OVERRIDE), Commands.run(action))
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
    return Commands.sequence(
            runGoal(V2_RedundancySuperstructureStates.OVERRIDE), Commands.run(action))
        .withTimeout(timeSeconds)
        .finallyDo(() -> setGoal(currentState));
  }

  /**
   * Runs the goal state and waits until a given condition becomes true.
   *
   * @param goal The desired superstructure state
   * @param condition The condition to wait for after running the goal
   * @return Combined command for running and waiting
   */
  public Command runGoalUntil(V2_RedundancySuperstructureStates goal, BooleanSupplier condition) {
    return Commands.sequence(runGoal(goal), Commands.waitUntil(condition));
  }

  public Command runGoalUntil(
      Supplier<V2_RedundancySuperstructureStates> goal, BooleanSupplier condition) {
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
   * Converts a ReefState (field-level enum) into a corresponding elevator goal and runs it.
   *
   * @param goal Supplier of ReefState
   * @return Command to move to the elevator position for the reef level
   */
  public Command runReefGoal(Supplier<ReefState> goal) {
    return runGoal(
        () -> {
          // Translate ReefState to superstructure state
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

  /**
   * Moves to a scoring position, executes the score action for a fixed time, then returns to pose.
   *
   * @param goal Supplier of the ReefState (target height/level)
   * @return Command to run the score cycle (pose → action → timeout → pose)
   */
  public Command runReefScoreGoal(Supplier<ReefState> goal) {
    // Run appropriate action sequence depending on reef level
    return runActionWithTimeout(
        () ->
            switch (goal.get()) {
              case L1 -> V2_RedundancySuperstructureStates.L1;
              case L2 -> V2_RedundancySuperstructureStates.L2;
              case L3 -> V2_RedundancySuperstructureStates.L3;
              case L4 -> V2_RedundancySuperstructureStates.L4;
              default -> V2_RedundancySuperstructureStates.STOW_DOWN;
            },
        () ->
            switch (goal.get()) {
              case L1 -> 0.8;
              case L2 -> 0.15;
              case L3 -> 0.15;
              case L4 -> 0.4;
              default -> 0;
            });
  }

  public Command l4PlusSequence() {
    return runActionWithTimeout(
        V2_RedundancySuperstructureStates.L4, V2_RedundancySuperstructureStates.SCORE_L4_PLUS, 0.5);
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
      Supplier<V2_RedundancySuperstructureStates> pose,
      Supplier<V2_RedundancySuperstructureStates> action,
      DoubleSupplier timeout) {
    return Commands.sequence(
            runGoal(action), // Run the action
            Commands.waitUntil(() -> atGoal()),
            Commands.defer(() -> Commands.waitSeconds(timeout.getAsDouble()), Set.of()))
        .finallyDo(() -> setGoal(pose.get())); // Return to original pose
  }

  public Command runActionWithTimeout(
      V2_RedundancySuperstructureStates pose,
      V2_RedundancySuperstructureStates action,
      double timeout) {
    return Commands.sequence(
            runGoal(action), // Run the action
            Commands.waitUntil(() -> atGoal()),
            Commands.waitSeconds(timeout),
            runGoal(pose))
        .finallyDo(() -> setGoal(pose)); // Return to original pose
  }

  /**
   * Smart overload that runs an action using a cached pose–action mapping, determining the pose
   * from the action.
   *
   * @param action The scoring or action state
   * @param timeout Timeout for the action duration
   * @return Command sequence to perform and recover from the action
   */
  public Command runActionWithTimeout(
      Supplier<V2_RedundancySuperstructureStates> action, DoubleSupplier timeout) {
    // Maps each action state to its corresponding pose state
    Map<V2_RedundancySuperstructureStates, V2_RedundancySuperstructureStates> actionPoseMap =
        new HashMap<>() {
          {
            put(V2_RedundancySuperstructureStates.SCORE_L1, V2_RedundancySuperstructureStates.L1);
            put(V2_RedundancySuperstructureStates.SCORE_L2, V2_RedundancySuperstructureStates.L2);
            put(V2_RedundancySuperstructureStates.SCORE_L3, V2_RedundancySuperstructureStates.L3);
            put(V2_RedundancySuperstructureStates.SCORE_L4, V2_RedundancySuperstructureStates.L4);
            put(
                V2_RedundancySuperstructureStates.SCORE_L4_PLUS,
                V2_RedundancySuperstructureStates.L4_PLUS);
            put(
                V2_RedundancySuperstructureStates.INTAKE_STATION,
                V2_RedundancySuperstructureStates.STOW_DOWN);
            put(
                V2_RedundancySuperstructureStates.SCORE_BARGE,
                V2_RedundancySuperstructureStates.BARGE);
            put(
                V2_RedundancySuperstructureStates.SCORE_PROCESSOR,
                V2_RedundancySuperstructureStates.PROCESSOR);
          }
        };

    // Reverse the map so we can look up the pose from action if needed
    Map<V2_RedundancySuperstructureStates, V2_RedundancySuperstructureStates> poseActionMap =
        actionPoseMap.entrySet().stream()
            .collect(Collectors.toMap(Map.Entry::getValue, Map.Entry::getKey));

    // Try to look up pose from action (either direction works)
    if (actionPoseMap.containsKey(action.get())) {
      return runActionWithTimeout(() -> actionPoseMap.get(action.get()), action, timeout);
    } else if (actionPoseMap.containsValue(action.get())) {
      return runActionWithTimeout(action, () -> poseActionMap.get(action.get()), timeout);
    } else return Commands.none(); // If action is not recognized, do nothing
  }

  public Command runActionWithTimeout(V2_RedundancySuperstructureStates action, double timeout) {
    // Maps each action state to its corresponding pose state
    Map<V2_RedundancySuperstructureStates, V2_RedundancySuperstructureStates> actionPoseMap =
        new HashMap<>() {
          {
            put(V2_RedundancySuperstructureStates.SCORE_L1, V2_RedundancySuperstructureStates.L1);
            put(V2_RedundancySuperstructureStates.SCORE_L2, V2_RedundancySuperstructureStates.L2);
            put(V2_RedundancySuperstructureStates.SCORE_L3, V2_RedundancySuperstructureStates.L3);
            put(V2_RedundancySuperstructureStates.SCORE_L4, V2_RedundancySuperstructureStates.L4);
            put(
                V2_RedundancySuperstructureStates.SCORE_L4_PLUS,
                V2_RedundancySuperstructureStates.L4_PLUS);
            put(
                V2_RedundancySuperstructureStates.INTAKE_STATION,
                V2_RedundancySuperstructureStates.STOW_DOWN);
            put(
                V2_RedundancySuperstructureStates.SCORE_BARGE,
                V2_RedundancySuperstructureStates.BARGE);
            put(
                V2_RedundancySuperstructureStates.SCORE_PROCESSOR,
                V2_RedundancySuperstructureStates.PROCESSOR);
          }
        };

    // Reverse the map so we can look up the pose from action if needed
    Map<V2_RedundancySuperstructureStates, V2_RedundancySuperstructureStates> poseActionMap =
        actionPoseMap.entrySet().stream()
            .collect(Collectors.toMap(Map.Entry::getValue, Map.Entry::getKey));

    // Try to look up pose from action (either direction works)
    if (actionPoseMap.containsKey(action)) {
      return runActionWithTimeout(actionPoseMap.get(action), action, timeout);
    } else if (actionPoseMap.containsValue(action)) {
      return runActionWithTimeout(action, poseActionMap.get(action), timeout);
    } else return Commands.none(); // If action is not recognized, do nothing
  }

  /**
   * Updates the superstructure to match the current OI-defined reef height.
   *
   * @return Command to move to elevator position state
   */
  public Command setPosition() {
    return runGoal(() -> getElevatorPosition()).withTimeout(0.02);
  }

  public boolean elevatorAtGoal() {
    return elevator.atGoal();
  }
}
