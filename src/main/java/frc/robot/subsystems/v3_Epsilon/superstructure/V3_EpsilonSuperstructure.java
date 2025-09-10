package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotMode;
import frc.robot.RobotState.ScoreSide;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureEdges.EdgeCommand;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureEdges.GamePieceEdge;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.Side;
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
 * The V3_EpsilonSuperstructure class manages the coordinated movement and state transitions of the
 * robot's major subsystems including elevator, manipulator, and intake.
 */
public class V3_EpsilonSuperstructure extends SubsystemBase {
  private final Graph<V3_EpsilonSuperstructureStates, EdgeCommand> graph;
  private final ElevatorFSM elevator;
  private final V3_EpsilonIntake intake;
  private final V3_EpsilonManipulator manipulator;

  /**
   * The previous, current, and next states of the superstructure. These are used to track the state
   * transitions and manage the command scheduling.
   */
  @Getter private V3_EpsilonSuperstructureStates previousState;

  /**
   * The current state of the superstructure, which is updated periodically based on the command
   * scheduling and state transitions.
   */
  @Getter private V3_EpsilonSuperstructureStates currentState;

  /**
   * The next state that the superstructure is transitioning to. This is determined by the command
   * scheduling and the current target state.
   */
  @Getter private V3_EpsilonSuperstructureStates nextState;

  /**
   * The target state that the superstructure is trying to achieve. This is set by the robot and
   * determines the next action to be taken.
   */
  @Getter private V3_EpsilonSuperstructureStates targetState;

  /** The command that is currently being executed to transition between states. */
  private EdgeCommand edgeCommand;

  /**
   * Constructs a V2_RedundancySuperstructure.
   *
   * @param elevator The elevator subsystem.
   * @param intake The intake subsystem.
   * @param manipulator The manipulator subsystem.
   */
  public V3_EpsilonSuperstructure(
      ElevatorFSM elevator, V3_EpsilonIntake intake, V3_EpsilonManipulator manipulator) {
    this.elevator = elevator;
    this.intake = intake;
    this.manipulator = manipulator;

    previousState = null;
    currentState = V3_EpsilonSuperstructureStates.START;
    nextState = null;

    targetState = V3_EpsilonSuperstructureStates.START;

    // Initialize the graph
    graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    for (V3_EpsilonSuperstructureStates vertex : V3_EpsilonSuperstructureStates.values()) {
      graph.addVertex(vertex);
    }

    // Add edges between states
    V3_EpsilonSuperstructureEdges.addEdges(graph, elevator, intake, manipulator);
  }

  /**
   * Periodic method that handles state transitions and subsystem updates. Updates robot state
   * variables and manages command scheduling based on current state.
   */
  @Override
  public void periodic() {
    manipulator.setArmSide(
        RobotState.getScoreSide().equals(ScoreSide.LEFT) ? Side.NEGATIVE : Side.POSITIVE);
    manipulator.setClearsElevator(
        elevator.getPositionMeters()
            > V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS() * 1.1);

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

    double armHeight =
        -manipulator.getArmAngle().rotateBy(new Rotation2d(-Math.PI / 2)).getSin()
                * V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS()
            + elevator.getPositionMeters();
    Logger.recordOutput(NTPrefixes.SUPERSTRUCTURE + "Arm Height", armHeight);
    Logger.recordOutput(NTPrefixes.SUPERSTRUCTURE + "Clears Base", 0.075 < armHeight);
    Logger.recordOutput(
        NTPrefixes.SUPERSTRUCTURE + "Clears Intake",
        intake.getPivotAngle().getDegrees() > 15
            || armHeight > 0.37
            || Math.abs(
                    manipulator.getArmAngle().rotateBy(new Rotation2d(-Math.PI / 2)).getCos()
                        * V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS())
                > 0.35);
  }

  /**
   * Updates the target state and handles command rescheduling for optimal path.
   *
   * @param goal New target state to achieve
   */
  private void setGoal(V3_EpsilonSuperstructureStates goal) {
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
  private Optional<V3_EpsilonSuperstructureStates> bfs(
      V3_EpsilonSuperstructureStates start, V3_EpsilonSuperstructureStates goal) {
    Map<V3_EpsilonSuperstructureStates, V3_EpsilonSuperstructureStates> parents = new HashMap<>();
    Queue<V3_EpsilonSuperstructureStates> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null);
    while (!queue.isEmpty()) {
      V3_EpsilonSuperstructureStates current = queue.poll();
      if (current == goal) break;
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        V3_EpsilonSuperstructureStates neighbor = graph.getEdgeTarget(edge);
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    if (!parents.containsKey(goal)) return Optional.empty();

    V3_EpsilonSuperstructureStates nextState = goal;
    while (!nextState.equals(start)) {
      V3_EpsilonSuperstructureStates parent = parents.get(nextState);
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
  private boolean isEdgeAllowed(EdgeCommand edge, V3_EpsilonSuperstructureStates goal) {
    return edge.getGamePieceEdge() == GamePieceEdge.UNCONSTRAINED
        || RobotState.isHasAlgae() == (edge.getGamePieceEdge() != GamePieceEdge.NO_ALGAE);
  }

  /** Resets the superstructure to initial auto state. */
  public void setAutoStart() {
    currentState = V3_EpsilonSuperstructureStates.START;
    nextState = null;
    targetState = V3_EpsilonSuperstructureStates.STOW_DOWN;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
  }

  /**
   * Maps current OI reef height to corresponding elevator position state.
   *
   * @return Appropriate superstructure state for current reef height
   */
  private V3_EpsilonSuperstructureStates getElevatorPosition() {
    switch (RobotState.getOIData().currentReefHeight()) {
      case STOW, CORAL_INTAKE -> {
        return V3_EpsilonSuperstructureStates.STOW_DOWN;
      }
      case L1 -> {
        return V3_EpsilonSuperstructureStates.L1;
      }
      case L2 -> {
        return V3_EpsilonSuperstructureStates.L2;
      }
      case L3 -> {
        return V3_EpsilonSuperstructureStates.L3;
      }
      case L4 -> {
        return V3_EpsilonSuperstructureStates.L4;
      }
      case ALGAE_INTAKE_BOTTOM -> {
        return V3_EpsilonSuperstructureStates.L2_ALGAE;
      }
      case ALGAE_INTAKE_TOP -> {
        return V3_EpsilonSuperstructureStates.L3_ALGAE;
      }
      case ALGAE_SCORE -> {
        return V3_EpsilonSuperstructureStates.BARGE;
      }
      default -> {
        return V3_EpsilonSuperstructureStates.START;
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
  public Command runGoal(V3_EpsilonSuperstructureStates goal) {
    return runOnce(() -> setGoal(goal));
  }

  /**
   * Returns a command that sets the superstructure to the goal provided by a supplier.
   *
   * @param goal Supplier providing the desired superstructure state
   * @return Command to run the goal
   */
  public Command runGoal(Supplier<V3_EpsilonSuperstructureStates> goal) {
    return runOnce(() -> setGoal(goal.get()));
  }

  /**
   * Checks whether the superstructure has reached its target state.
   *
   * @return true if current state matches the target state
   */
  @AutoLogOutput(key = NTPrefixes.SUPERSTRUCTURE + "At Goal")
  public boolean atGoal() {
    return currentState == targetState
        && elevator.atGoal()
        && intake.pivotAtGoal()
        && manipulator.armAtGoal();
  }

  public Command waitUntilAtGoal() {
    return Commands.sequence(Commands.waitSeconds(.02), Commands.waitUntil(() -> atGoal()));
  }

  /**
   * Runs a temporary override action, returning to a previous goal after.
   *
   * @param action The override action to perform
   * @param oldGoal The goal to return to after override
   * @return Command that runs the override and resumes the old goal
   */
  public Command override(Runnable action) {
    return Commands.sequence(runGoal(V3_EpsilonSuperstructureStates.OVERRIDE), Commands.run(action))
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
  public Command runGoalUntil(V3_EpsilonSuperstructureStates goal, BooleanSupplier condition) {
    return Commands.sequence(runGoal(goal), Commands.waitUntil(condition));
  }

  public Command runGoalUntil(
      Supplier<V3_EpsilonSuperstructureStates> goal, BooleanSupplier condition) {
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
              return V3_EpsilonSuperstructureStates.L1;
            case L2:
              return V3_EpsilonSuperstructureStates.L2;
            case L3:
              return V3_EpsilonSuperstructureStates.L3;
            case L4:
              return V3_EpsilonSuperstructureStates.L4;
            default:
              return V3_EpsilonSuperstructureStates.STOW_DOWN;
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
              case L1 -> V3_EpsilonSuperstructureStates.L1;
              case L2 -> V3_EpsilonSuperstructureStates.L2;
              case L3 -> V3_EpsilonSuperstructureStates.L3;
              case L4 -> V3_EpsilonSuperstructureStates.L4;
              default -> V3_EpsilonSuperstructureStates.STOW_DOWN;
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

  /**
   * Runs an action by going to a pose, performing the action, waiting, and returning.
   *
   * @param pose The pose to return to after action
   * @param action The action to perform
   * @param timeout How long to wait during the action phase (in seconds)
   * @return Full command sequence
   */
  public Command runActionWithTimeout(
      Supplier<V3_EpsilonSuperstructureStates> pose,
      Supplier<V3_EpsilonSuperstructureStates> action,
      DoubleSupplier timeout) {
    return Commands.sequence(
            runGoal(action), // Run the action
            waitUntilAtGoal(),
            Commands.defer(() -> Commands.waitSeconds(timeout.getAsDouble()), Set.of()))
        .finallyDo(() -> setGoal(pose.get())); // Return to original pose
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
      V3_EpsilonSuperstructureStates pose, V3_EpsilonSuperstructureStates action, double timeout) {
    return Commands.sequence(
            runGoal(action), // Run the action
            waitUntilAtGoal(),
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
      Supplier<V3_EpsilonSuperstructureStates> action, DoubleSupplier timeout) {
    // Maps each action state to its corresponding pose state
    Map<V3_EpsilonSuperstructureStates, V3_EpsilonSuperstructureStates> actionPoseMap =
        new HashMap<>() {
          {
            put(V3_EpsilonSuperstructureStates.L1_SCORE, V3_EpsilonSuperstructureStates.L1);
            put(V3_EpsilonSuperstructureStates.L2_SCORE, V3_EpsilonSuperstructureStates.L2);
            put(V3_EpsilonSuperstructureStates.L3_SCORE, V3_EpsilonSuperstructureStates.L3);
            put(V3_EpsilonSuperstructureStates.L4_SCORE, V3_EpsilonSuperstructureStates.L4);
            put(V3_EpsilonSuperstructureStates.BARGE_SCORE, V3_EpsilonSuperstructureStates.BARGE);
            put(
                V3_EpsilonSuperstructureStates.PROCESSOR_SCORE,
                V3_EpsilonSuperstructureStates.PROCESSOR);
          }
        };

    // Reverse the map so we can look up the pose from action if needed
    Map<V3_EpsilonSuperstructureStates, V3_EpsilonSuperstructureStates> poseActionMap =
        actionPoseMap.entrySet().stream()
            .collect(Collectors.toMap(Map.Entry::getValue, Map.Entry::getKey));

    // Try to look up pose from action (either direction works)
    if (actionPoseMap.containsKey(action.get())) {
      return runActionWithTimeout(() -> actionPoseMap.get(action.get()), action, timeout);
    } else if (actionPoseMap.containsValue(action.get())) {
      return runActionWithTimeout(action, () -> poseActionMap.get(action.get()), timeout);
    } else return Commands.none(); // If action is not recognized, do nothing
  }

  /**
   * Smart overload that runs an action using a cached pose–action mapping, determining the pose
   * from the action.
   *
   * @param action The scoring or action state
   * @param timeout Timeout for the action duration
   * @return Command sequence to perform and recover from the action
   */
  public Command runActionWithTimeout(V3_EpsilonSuperstructureStates action, double timeout) {
    // Maps each action state to its corresponding pose state
    Map<V3_EpsilonSuperstructureStates, V3_EpsilonSuperstructureStates> actionPoseMap =
        new HashMap<>() {
          {
            put(V3_EpsilonSuperstructureStates.L1_SCORE, V3_EpsilonSuperstructureStates.L1);
            put(V3_EpsilonSuperstructureStates.L2_SCORE, V3_EpsilonSuperstructureStates.L2);
            put(V3_EpsilonSuperstructureStates.L3_SCORE, V3_EpsilonSuperstructureStates.L3);
            put(V3_EpsilonSuperstructureStates.L4_SCORE, V3_EpsilonSuperstructureStates.L4);
            put(V3_EpsilonSuperstructureStates.BARGE_SCORE, V3_EpsilonSuperstructureStates.BARGE);
            put(
                V3_EpsilonSuperstructureStates.PROCESSOR_SCORE,
                V3_EpsilonSuperstructureStates.PROCESSOR);
          }
        };

    // Reverse the map so we can look up the pose from action if needed
    Map<V3_EpsilonSuperstructureStates, V3_EpsilonSuperstructureStates> poseActionMap =
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

  /**
   * Checks if the elevator is at its goal position.
   *
   * @return True if the elevator is at its goal, false otherwise.
   */
  public boolean elevatorAtGoal() {
    return elevator.atGoal();
  }

  public Command allTransition() {
    Command all = runGoal(V3_EpsilonSuperstructureStates.STOW_DOWN);
    for (var source : V3_EpsilonSuperstructureStates.values()) {
      for (var sink : V3_EpsilonSuperstructureStates.values()) {
        if (source == sink) continue;
        var edge = graph.getEdge(source, sink);
        if (edge != null) {

          if (source != V3_EpsilonSuperstructureStates.START
              && sink != V3_EpsilonSuperstructureStates.START
              && source != V3_EpsilonSuperstructureStates.OVERRIDE) {
            all =
                all.andThen(
                    runGoal(sink),
                    runOnce(() -> System.out.println("Initial Pose:" + sink)),
                    Commands.waitSeconds(2));
            all =
                all.andThen(
                    runGoal(source),
                    runOnce(() -> System.out.println("Final Pose:" + source)),
                    Commands.waitSeconds(2));
          }
        }
      }
    }
    return all;
  }
}
