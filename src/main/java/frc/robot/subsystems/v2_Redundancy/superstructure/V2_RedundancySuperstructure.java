package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancyStates.Edge;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancyStates.SuperstructureStates;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeExtensionState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
import frc.robot.util.NTPrefixes;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancySuperstructure extends SubsystemBase {

  private final Graph<SuperstructureStates, EdgeCommand> graph;
  private final V2_RedundancyElevator elevator;
  private final V2_RedundancyFunnel funnel;
  private final V2_RedundancyManipulator manipulator;
  private final V2_RedundancyIntake intake;

  @Getter private SuperstructureStates previousState;
  @Getter private SuperstructureStates currentState;
  @Getter private SuperstructureStates nextState;

  @Getter private SuperstructureStates targetState;
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
    currentState = SuperstructureStates.START;
    nextState = null;

    targetState = SuperstructureStates.START;

    // Initialize the graph
    graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    for (SuperstructureStates vertex : SuperstructureStates.values()) {
      graph.addVertex(vertex);
    }

    // Add edges between states
    V2_RedundancyStates.createEdges();
    addEdges(V2_RedundancyStates.NoneEdges, AlgaeEdge.NONE);
    addEdges(V2_RedundancyStates.NoAlgaeEdges, AlgaeEdge.NO_ALGAE);
    addEdges(V2_RedundancyStates.AlgaeEdges, AlgaeEdge.ALGAE);
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
    RobotState.setIntakingCoral(targetState == SuperstructureStates.INTAKE);
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
        if (targetState != SuperstructureStates.OVERRIDE) {
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

  public Command runGoal(SuperstructureStates goal) {
    return runOnce(() -> setGoal(goal));
  }

  @AutoLogOutput(key = NTPrefixes.SUPERSTRUCTURE + "At Goal")
  public boolean atGoal() {
    return currentState == targetState;
  }

  public Command runGoal(Supplier<SuperstructureStates> goal) {
    return runOnce(() -> setGoal(goal.get()));
  }

  public Command override(Runnable action, SuperstructureStates oldGoal) {
    return Commands.sequence(runGoal(SuperstructureStates.OVERRIDE), Commands.run(action))
        .finallyDo(() -> setGoal(oldGoal));
  }

  public Command runGoalUntil(SuperstructureStates goal, BooleanSupplier condition) {
    return Commands.sequence(runGoal(goal), Commands.waitUntil(condition));
  }

  public Command runReefGoal(Supplier<ReefState> goal) {
    return runGoal(
        () -> {
          switch (goal.get()) {
            case L1:
              return SuperstructureStates.L1;
            case L2:
              return SuperstructureStates.L2;
            case L3:
              return SuperstructureStates.L3;
            case L4:
              return SuperstructureStates.L4;
            case L4_PLUS:
              return SuperstructureStates.L4_PLUS;
            default:
              return SuperstructureStates.STOW_DOWN;
          }
        });
  }

  public Command runReefScoreGoal(Supplier<ReefState> goal) {
    switch (goal.get()) {
      case L1:
        return runActionWithTimeout(SuperstructureStates.L1, SuperstructureStates.SCORE_L1, 0.8);
      case L2:
        return runActionWithTimeout(SuperstructureStates.L2, SuperstructureStates.SCORE_L2, 0.15);
      case L3:
        return runActionWithTimeout(SuperstructureStates.L3, SuperstructureStates.SCORE_L3, 0.15);
      case L4:
        return runActionWithTimeout(SuperstructureStates.L4, SuperstructureStates.SCORE_L4, 0.4);
      case L4_PLUS:
        return runActionWithTimeout(
            SuperstructureStates.L4_PLUS, SuperstructureStates.SCORE_L4_PLUS, 0.5);
      default:
        return Commands.none();
    }
  }

  public Command runActionWithTimeout(
      SuperstructureStates pose, SuperstructureStates action, double timeout) {
    return Commands.sequence(
        runGoal(pose),
        Commands.waitUntil(() -> atGoal()),
        runGoal(action),
        Commands.waitSeconds(timeout),
        runGoal(pose));
  }

  private Command getEdgeCommand(SuperstructureStates from, SuperstructureStates to) {
    V2_RedundancySuperstructurePose pose = to.getPose();

    if (from == SuperstructureStates.INTAKE_FLOOR) {
      return Commands.parallel(
          pose.asCommand(elevator, manipulator, funnel, intake),
          Commands.run(() -> intake.setRollerGoal(IntakeRollerState.OUTTAKE))
              .withTimeout(1)
              .andThen(Commands.runOnce(() -> intake.setRollerGoal(IntakeRollerState.STOP))));
    }

    if ((to == SuperstructureStates.INTAKE_REEF_L2 || to == SuperstructureStates.INTAKE_REEF_L3)
        && (from != SuperstructureStates.STOW_UP || from != SuperstructureStates.BARGE)) {
      return Commands.sequence(
          pose.setElevatorHeight(elevator)
              .alongWith(
                  Commands.runOnce(() -> manipulator.setAlgaeArmGoal(ArmState.STOW_DOWN))
                      .alongWith(manipulator.waitUntilAlgaeArmAtGoal()))
              .alongWith(pose.setFunnelState(funnel).alongWith(pose.setIntakeState(intake))),
          pose.setArmState(manipulator));
    }

    if (to == SuperstructureStates.FLOOR_ACQUISITION) {
      return Commands.deadline(
          pose.asCommand(elevator, manipulator, funnel, intake),
          Commands.runOnce(() -> intake.setRollerGoal(IntakeRollerState.INTAKE)));
    }

    if (to == SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM
        || (from == SuperstructureStates.FLOOR_ACQUISITION && to == SuperstructureStates.STOW_DOWN)
        || to == SuperstructureStates.STOW_UP) {
      return pose.setArmState(manipulator)
          .andThen(
              pose.setIntakeState(intake)
                  .alongWith(pose.setElevatorHeight(elevator))
                  .alongWith(pose.setFunnelState(funnel)));
    }
    if (to == SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR) {
      return pose.setElevatorHeight(elevator)
          .andThen(
              pose.setIntakeState(intake)
                  .alongWith(pose.setArmState(manipulator), pose.setFunnelState(funnel)));
    }
    if (to == SuperstructureStates.L1 && from == SuperstructureStates.SCORE_L1) {
      return pose.asCommand(elevator, manipulator, funnel, intake)
          .andThen(Commands.runOnce(() -> intake.setExtensionGoal(IntakeExtensionState.L1_EXT)));
    }

    return pose.asCommand(elevator, manipulator, funnel, intake); // does all action in paralell
  }

  private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureStates goal) {
    return (!edge.isRestricted() || goal == graph.getEdgeTarget(edge))
        && (edge.getAlgaeEdgeType() == AlgaeEdge.NONE
            || RobotState.isHasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE));
  }

  public void setAutoStart() {
    currentState = SuperstructureStates.START;
    nextState = null;
    targetState = SuperstructureStates.STOW_DOWN;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
  }

  private Optional<SuperstructureStates> bfs(
      SuperstructureStates start, SuperstructureStates goal) {
    // Map to track the parent of each visited node
    Map<SuperstructureStates, SuperstructureStates> parents = new HashMap<>();
    Queue<SuperstructureStates> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      SuperstructureStates current = queue.poll();
      // Check if we've reached the goal
      if (current == goal) {
        break;
      }
      // Process valid neighbors
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        SuperstructureStates neighbor = graph.getEdgeTarget(edge);
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
    SuperstructureStates nextState = goal;
    while (!nextState.equals(start)) {
      SuperstructureStates parent = parents.get(nextState);
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

  private void setGoal(SuperstructureStates goal) {
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

  private void addEdge(SuperstructureStates from, SuperstructureStates to, AlgaeEdge algaeEdge) {
    graph.addEdge(
        from,
        to,
        EdgeCommand.builder()
            .command(getEdgeCommand(from, to))
            .algaeEdgeType(algaeEdge)
            .restricted(false)
            .build());
  }

  private void addEdges(List<Edge> edges, AlgaeEdge type) {
    for (Edge edge : edges) {
      addEdge(edge.from(), edge.to(), type);
    }
  }

  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final boolean restricted = false;
    @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  }

  private enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }

  private SuperstructureStates getElevatorPosition() {
    switch (RobotState.getOIData().currentReefHeight()) {
      case STOW, CORAL_INTAKE -> {
        return SuperstructureStates.STOW_DOWN;
      }
      case L1 -> {
        return SuperstructureStates.L1;
      }
      case L2 -> {
        return SuperstructureStates.L2;
      }
      case L3 -> {
        return SuperstructureStates.L3;
      }
      case L4 -> {
        return SuperstructureStates.L4;
      }
      default -> {
        return SuperstructureStates.START;
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
