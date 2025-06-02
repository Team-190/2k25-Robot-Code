package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.subsystems.v2_Redundancy.superstructure.SuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

public class Superstructure extends SubsystemBase {

  private final Graph<SuperstructureStates, EdgeCommand> graph;
  private final V2_RedundancyElevator elevator;
  private final V2_RedundancyFunnel funnel;
  private final V2_RedundancyManipulator manipulator;
  private final V2_RedundancyIntake intake;

  private SuperstructureStates previousState;
  private SuperstructureStates currentState;
  private SuperstructureStates nextState;

  private SuperstructureStates targetState;
  private EdgeCommand edgeCommand;

  public enum SuperstructureStates {
    START("START", new SubsystemPoses()),
    STOW_DOWN(
        "STOW DOWN",
        new SubsystemPoses(
            ReefState.STOW, ArmState.STOW_DOWN,
            IntakeState.STOW, FunnelState.OPENED)),
    INTAKE(
        "INTAKE CORAL",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.CORAL_INTAKE_VOLTS().get(),
            12.0,
            0.0)),
    L1(
        "L1 CORAL SETPOINT",
        new SubsystemPoses(ReefState.L1, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    L2(
        "L2 CORAL SETPOINT",
        new SubsystemPoses(ReefState.L2, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    L3(
        "L3 CORAL SETPOINT",
        new SubsystemPoses(ReefState.L3, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    L4(
        "L4 CORAL SETPOINT",
        new SubsystemPoses(ReefState.L4, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    L4_PLUS(
        "L4+ CORAL SETPOINT",
        new SubsystemPoses(
            ReefState.L4_PLUS, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    SCORE_L1(
        "L1 CORAL SCORE",
        List.of(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.L1_VOLTS().get(), 0.0, 0.0)),
    SCORE_L2(
        "L2 CORAL SCORE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_CORAL_VOLTS().get(), 0.0, 0.0)),
    SCORE_L3(
        "L3 CORAL SCORE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_CORAL_VOLTS().get(), 0.0, 0.0)),
    SCORE_L4(
        "L4 CORAL SCORE",
        List.of(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.L4_VOLTS().get(), 0.0, 0.0)),
    SCORE_L4_PLUS(
        "L4+ CORAL SCORE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_CORAL_VOLTS().get(), 0.0, 0.0)),
    INTERMEDIATE_WAIT_FOR_ELEVATOR(
        "WAIT FOR ELEVATOR",
        new SubsystemPoses(
            ReefState.ALGAE_MID,
            ArmState.STOW_DOWN,
            IntakeState.STOW,
            FunnelState.OPENED)), // TODO: Fix this
    // INTERMEDIATE_WAIT_FOR_ARM,
    STOW_UP(
        "STOW UP",
        new SubsystemPoses(ReefState.STOW, ArmState.STOW_UP, IntakeState.STOW, FunnelState.OPENED)),
    FLOOR_ACQUISITION(
        "FLOOR ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.STOW, ArmState.FLOOR_INTAKE, IntakeState.INTAKE, FunnelState.OPENED)),
    REEF_ACQUISITION_L2(
        "L2 ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_BOTTOM,
            ArmState.REEF_INTAKE,
            IntakeState.STOW,
            FunnelState.OPENED)),
    REEF_ACQUISITION_L3(
        "L3 ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_TOP,
            ArmState.REEF_INTAKE,
            IntakeState.STOW,
            FunnelState.OPENED)),
    BARGE(
        "BARGE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_SCORE, ArmState.STOW_UP, IntakeState.STOW, FunnelState.CLOSED)),
    PROCESSOR(
        "PROCESSOR SETPOINT",
        new SubsystemPoses(
            ReefState.STOW, ArmState.PROCESSOR, IntakeState.STOW, FunnelState.CLOSED)),
    INTAKE_FLOOR(
        "INTAKE FLOOR",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get(),
            0.0,
            6.0)),
    INTAKE_REEF_L2(
        "L2 ALGAE INTAKE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get(),
            0.0,
            0.0)),
    INTAKE_REEF_L3(
        "L3 ALGAE INTAKE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get(),
            0.0,
            0.0)),
    DROP_REEF_L2(
        "DROP L2 ALGAE",
        List.of(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.REMOVE_ALGAE().get(), 0.0, 0.0)),
    DROP_REEF_L3(
        "DROP L3 ALGAE",
        List.of(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.REMOVE_ALGAE().get(), 0.0, 0.0)),
    SCORE_BARGE(
        "SCORE BARGE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_ALGAE_VOLTS().get(), 0.0, 0.0)),
    SCORE_PROCESSOR(
        "SCORE PROCESSOR",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_ALGAE_VOLTS().get(), 0.0, 0.0)),
    ;
    private final String name;
    private SubsystemPoses subsystemPoses;
    private List<Double> voltages;

    private SuperstructureStates(String name, SubsystemPoses poses) {
      this.name = name;
      this.subsystemPoses = poses;
      this.voltages = null;
    }

    private SuperstructureStates(String name, List<Double> voltages) {
      this.name = name;
      this.subsystemPoses = null;
      this.voltages = voltages;
    }

    public SuperstructureState createPose(
        V2_RedundancyElevator elevator,
        V2_RedundancyFunnel funnel,
        V2_RedundancyManipulator manipulator,
        V2_RedundancyIntake intake) {
      if (subsystemPoses != null) {
        return new SuperstructurePose(name, subsystemPoses, elevator, funnel, manipulator, intake);
      } else {
        return new SuperstructureAction(
            name,
            voltages.get(0),
            voltages.get(1),
            voltages.get(2),
            elevator,
            manipulator,
            funnel,
            intake);
      }
    }

    @Override
    public String toString() {
      return name;
    }
  }

  private Superstructure(
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

    graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    for (SuperstructureStates vertex : SuperstructureStates.values()) {
      graph.addVertex(vertex);
    }
  }

  private void addEdge(SuperstructureStates from, SuperstructureStates to) {
    addEdge(from, to, AlgaeEdge.NONE);
  }

  private void addEdge(SuperstructureStates from, SuperstructureStates to, AlgaeEdge algaeEdge) {
    addEdge(from, to, false, algaeEdge, false);
  }

  private void addEdge(
      SuperstructureStates from,
      SuperstructureStates to,
      boolean reverse,
      AlgaeEdge algaeEdge,
      boolean restricted) {
    graph.addEdge(
        from,
        to,
        EdgeCommand.builder()
            .command(getEdgeCommand(from, to))
            .algaeEdgeType(algaeEdge)
            .restricted(restricted)
            .build());
    if (reverse) {
      graph.addEdge(
          to,
          from,
          EdgeCommand.builder()
              .command(getEdgeCommand(to, from))
              .algaeEdgeType(algaeEdge)
              .restricted(restricted)
              .build());
    }
  }

  public Command runGoal(SuperstructureStates goal) {
    return runOnce(() -> setGoal(goal)).andThen(Commands.idle(this));
  }

  public Command runGoal(Supplier<SuperstructureStates> goal) {
    return run(() -> setGoal(goal.get()));
  }

  private Command getEdgeCommand(SuperstructureStates from, SuperstructureStates to) {
    return Commands.none();
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
    this.targetState = goal;

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
                  currentState = nextState;
                  nextState = temp;
                }
              });
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
}
