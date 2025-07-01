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

public class V2_RedundancySuperstructure extends SubsystemBase {

  private final Graph<V2_RedundancySuperstructureStates, EdgeCommand> graph;
  private final V2_RedundancyElevator elevator;
  private final V2_RedundancyFunnel funnel;
  private final V2_RedundancyManipulator manipulator;
  private final V2_RedundancyIntake intake;

  @Getter private V2_RedundancySuperstructureStates previousState;
  @Getter private V2_RedundancySuperstructureStates currentState;
  @Getter private V2_RedundancySuperstructureStates nextState;

  @Getter private V2_RedundancySuperstructureStates targetState;
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

  private void stopActions() {
    funnel.setRollerGoal(FunnelRollerState.STOP);
    manipulator.runManipulator(ManipulatorRollerState.STOP);
    intake.setRollerGoal(IntakeRollerState.STOP);
  }

  @Override
  public void periodic() {
    elevator.periodic();
    funnel.periodic();
    manipulator.periodic();
    intake.periodic();

    // Set RobotState variables
    RobotState.setIntakingCoral(targetState == V2_RedundancySuperstructureStates.INTAKE);
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

  public Command runGoal(V2_RedundancySuperstructureStates goal) {
    return runOnce(() -> setGoal(goal));
  }

  @AutoLogOutput(key = NTPrefixes.SUPERSTRUCTURE + "At Goal")
  public boolean atGoal() {
    return currentState == targetState;
  }

  public Command runGoal(Supplier<V2_RedundancySuperstructureStates> goal) {
    return runOnce(() -> setGoal(goal.get()));
  }

  public Command override(Runnable action, V2_RedundancySuperstructureStates oldGoal) {
    return Commands.sequence(
            runGoal(V2_RedundancySuperstructureStates.OVERRIDE), Commands.run(action))
        .finallyDo(() -> setGoal(oldGoal));
  }

  public Command runGoalUntil(V2_RedundancySuperstructureStates goal, BooleanSupplier condition) {
    return Commands.sequence(runGoal(goal), Commands.waitUntil(condition));
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

  private boolean isEdgeAllowed(EdgeCommand edge, V2_RedundancySuperstructureStates goal) {
    return edge.getAlgaeEdgeType() == AlgaeEdge.NONE
        || RobotState.isHasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE);
  }

  public void setAutoStart() {
    currentState = V2_RedundancySuperstructureStates.START;
    nextState = null;
    targetState = V2_RedundancySuperstructureStates.STOW_DOWN;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
  }

  private Optional<V2_RedundancySuperstructureStates> bfs(
      V2_RedundancySuperstructureStates start, V2_RedundancySuperstructureStates goal) {
    // Map to track the parent of each visited node
    Map<V2_RedundancySuperstructureStates, V2_RedundancySuperstructureStates> parents =
        new HashMap<>();
    Queue<V2_RedundancySuperstructureStates> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      V2_RedundancySuperstructureStates current = queue.poll();
      // Check if we've reached the goal
      if (current == goal) {
        break;
      }
      // Process valid neighbors
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        V2_RedundancySuperstructureStates neighbor = graph.getEdgeTarget(edge);
        // Only process unvisited neighbors
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    // Reconstruct the path to the goal if found
    if (!parents.containsKey(goal)) {
      return Optional.empty(); // Goal not reachable
    }

    // Trace back the path from goal to start
    V2_RedundancySuperstructureStates nextState = goal;
    while (!nextState.equals(start)) {
      V2_RedundancySuperstructureStates parent = parents.get(nextState);
      if (parent == null) {
        return Optional.empty(); // No valid path found
      } else if (parent.equals(start)) {
        // Return the edge from start to the next node
        return Optional.of(nextState);
      }
      nextState = parent;
    }
    return Optional.of(nextState);
  }

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

  public Command setPosition() {
    return runGoal(() -> getElevatorPosition()).withTimeout(0.02);
  }

  public Command runPreviousState() {
    return runGoal(() -> previousState).withTimeout(0.02);
  }
}
