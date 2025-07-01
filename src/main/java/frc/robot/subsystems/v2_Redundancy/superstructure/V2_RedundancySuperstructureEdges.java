package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeExtensionState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;

public class V2_RedundancySuperstructureEdges {

  public static final ArrayList<Edge> NoneEdges = new ArrayList<>();
  public static final ArrayList<Edge> AlgaeEdges = new ArrayList<>();
  public static final ArrayList<Edge> NoAlgaeEdges = new ArrayList<>();

  public record Edge(V2_RedundancySuperstructureStates from, V2_RedundancySuperstructureStates to) {
    public Edge(V2_RedundancySuperstructureStates from, V2_RedundancySuperstructureStates to) {
      this.from = from;
      this.to = to;
    }

    @Override
    public String toString() {
      return from + " -> " + to;
    }
  }

  public enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }

  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  }

  private static Command getEdgeCommand(
      V2_RedundancySuperstructureStates from,
      V2_RedundancySuperstructureStates to,
      V2_RedundancyElevator elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
    V2_RedundancySuperstructurePose pose = to.getPose();

    if (from == V2_RedundancySuperstructureStates.INTAKE_FLOOR) {
      return Commands.parallel(
          pose.asCommand(elevator, manipulator, funnel, intake),
          Commands.run(() -> intake.setRollerGoal(IntakeRollerState.OUTTAKE))
              .withTimeout(1)
              .andThen(Commands.runOnce(() -> intake.setRollerGoal(IntakeRollerState.STOP))));
    }

    if ((to == V2_RedundancySuperstructureStates.INTAKE_REEF_L2
            || to == V2_RedundancySuperstructureStates.INTAKE_REEF_L3)
        && (from != V2_RedundancySuperstructureStates.STOW_UP
            || from != V2_RedundancySuperstructureStates.BARGE)) {
      return Commands.sequence(
          pose.setElevatorHeight(elevator)
              .alongWith(
                  Commands.runOnce(() -> manipulator.setAlgaeArmGoal(ArmState.STOW_DOWN))
                      .alongWith(manipulator.waitUntilAlgaeArmAtGoal()))
              .alongWith(pose.setFunnelState(funnel).alongWith(pose.setIntakeState(intake))),
          pose.setArmState(manipulator));
    }

    if (to == V2_RedundancySuperstructureStates.FLOOR_ACQUISITION) {
      return Commands.deadline(
          pose.asCommand(elevator, manipulator, funnel, intake),
          Commands.runOnce(() -> intake.setRollerGoal(IntakeRollerState.INTAKE)));
    }

    if (to == V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM
        || (from == V2_RedundancySuperstructureStates.FLOOR_ACQUISITION
            && to == V2_RedundancySuperstructureStates.STOW_DOWN)
        || to == V2_RedundancySuperstructureStates.STOW_UP) {
      return pose.setArmState(manipulator)
          .andThen(
              pose.setIntakeState(intake)
                  .alongWith(pose.setElevatorHeight(elevator))
                  .alongWith(pose.setFunnelState(funnel)));
    }
    if (to == V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR) {
      return pose.setElevatorHeight(elevator)
          .andThen(
              pose.setIntakeState(intake)
                  .alongWith(pose.setArmState(manipulator), pose.setFunnelState(funnel)));
    }
    if (to == V2_RedundancySuperstructureStates.L1
        && from == V2_RedundancySuperstructureStates.SCORE_L1) {
      return pose.asCommand(elevator, manipulator, funnel, intake)
          .andThen(Commands.runOnce(() -> intake.setExtensionGoal(IntakeExtensionState.L1_EXT)));
    }

    return pose.asCommand(
        elevator, manipulator, funnel, intake); // does all subsystem poses in paralell
  }

  private static void createEdges() {

    // CORAL-RELATED STATES
    List<V2_RedundancySuperstructureStates> coralLevels =
        List.of(
            V2_RedundancySuperstructureStates.L1,
            V2_RedundancySuperstructureStates.L2,
            V2_RedundancySuperstructureStates.L3,
            V2_RedundancySuperstructureStates.L4);

    // stow_down <-> each coral level (bidirectional, no algae)
    for (V2_RedundancySuperstructureStates level : coralLevels) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.STOW_DOWN, level));
      NoAlgaeEdges.add(new Edge(level, V2_RedundancySuperstructureStates.STOW_DOWN));
    }

    // every coral level <-> every other coral level (no algae)
    for (V2_RedundancySuperstructureStates from : coralLevels) {
      for (V2_RedundancySuperstructureStates to : coralLevels) {
        if (from != to) {
          NoneEdges.add(new Edge(from, to));
        }
      }
    }

    // Misellaneous L4+ transitions
    NoneEdges.add(
        new Edge(V2_RedundancySuperstructureStates.L4_PLUS, V2_RedundancySuperstructureStates.L4));
    NoneEdges.add(
        new Edge(V2_RedundancySuperstructureStates.L4, V2_RedundancySuperstructureStates.L4_PLUS));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.L4_PLUS,
            V2_RedundancySuperstructureStates.STOW_DOWN));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            V2_RedundancySuperstructureStates.L4_PLUS));

    // each “level” → its scoring state (one‑way, no algae)
    Map<V2_RedundancySuperstructureStates, V2_RedundancySuperstructureStates> coralScoreMap =
        Map.of(
            V2_RedundancySuperstructureStates.L1, V2_RedundancySuperstructureStates.SCORE_L1,
            V2_RedundancySuperstructureStates.L2, V2_RedundancySuperstructureStates.SCORE_L2,
            V2_RedundancySuperstructureStates.L3, V2_RedundancySuperstructureStates.SCORE_L3,
            V2_RedundancySuperstructureStates.L4, V2_RedundancySuperstructureStates.SCORE_L4,
            V2_RedundancySuperstructureStates.L4_PLUS,
                V2_RedundancySuperstructureStates.SCORE_L4_PLUS);
    coralScoreMap.forEach(
        (level, score) -> {
          NoAlgaeEdges.add(new Edge(score, level));
          NoAlgaeEdges.add(new Edge(level, score));
        });

    // each coral level → FLOOR_ACQUISITION and → INTERMEDIATE_WAIT_FOR_ELEVATOR (one‑way)
    for (V2_RedundancySuperstructureStates level :
        List.of(V2_RedundancySuperstructureStates.L1, V2_RedundancySuperstructureStates.L2)) {
      for (V2_RedundancySuperstructureStates target :
          List.of(
              V2_RedundancySuperstructureStates.FLOOR_ACQUISITION,
              V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR)) {
        NoneEdges.add(new Edge(level, target));
      }
    }

    for (V2_RedundancySuperstructureStates level :
        List.of(V2_RedundancySuperstructureStates.L3, V2_RedundancySuperstructureStates.L4)) {
      for (V2_RedundancySuperstructureStates target :
          List.of(
              V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
              V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3,
              V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM)) {
        NoneEdges.add(new Edge(level, target));
      }
    }

    // INTAKE_CORAL ↔ STOW_DOWN (bidirectional, no algae)
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_DOWN, V2_RedundancySuperstructureStates.INTAKE));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE, V2_RedundancySuperstructureStates.STOW_DOWN));

    // INTERMEDIATE_WAIT_FOR_ELEVATOR TRANSITIONS (all one‑way, no algae)
    List<V2_RedundancySuperstructureStates> iveDestinations =
        List.of(
            V2_RedundancySuperstructureStates.STOW_UP,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3,
            V2_RedundancySuperstructureStates.BARGE,
            V2_RedundancySuperstructureStates.PROCESSOR);
    for (V2_RedundancySuperstructureStates dest : iveDestinations) {
      AlgaeEdges.add(
          new Edge(V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR, dest));
    }

    // INTERMEDIATE_WAIT_FOR_ARM TRANSITIONS (one‑way, no algae)
    List<V2_RedundancySuperstructureStates> iwaDestinations =
        List.of(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.L1,
            V2_RedundancySuperstructureStates.L2,
            V2_RedundancySuperstructureStates.FLOOR_ACQUISITION);
    for (V2_RedundancySuperstructureStates dest : iwaDestinations) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM, dest));
    }

    // STOW_UP → multiple targets (two, algae)
    List<V2_RedundancySuperstructureStates> stowUpDestinations =
        List.of(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            V2_RedundancySuperstructureStates.BARGE,
            V2_RedundancySuperstructureStates.PROCESSOR);
    for (V2_RedundancySuperstructureStates dest : stowUpDestinations) {
      AlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.STOW_UP, dest));
      AlgaeEdges.add(new Edge(dest, V2_RedundancySuperstructureStates.STOW_UP));
    }

    // FLOOR_ACQUISITION → multiple targets (one‑way, no algae)
    List<V2_RedundancySuperstructureStates> floorAcqDest =
        List.of(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.INTAKE_FLOOR);
    for (V2_RedundancySuperstructureStates dest : floorAcqDest) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.FLOOR_ACQUISITION, dest));
    }
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.FLOOR_ACQUISITION,
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR));

    // REEF-RELATED ACQUISITION STATES (using algaeMap style)
    // Define “from → to” for algae acquisition (one‑way with ALGAE), then add reverse with
    // NO_ALGAE
    Map<V2_RedundancySuperstructureStates, List<V2_RedundancySuperstructureStates>>
        reefMap = // Algae states here are probably wrong
        Map.of(
                V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
                    List.of(
                        V2_RedundancySuperstructureStates.INTAKE_REEF_L2, // no alg
                        V2_RedundancySuperstructureStates.DROP_REEF_L2, // no alg
                        V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM, // none
                        V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3, // none
                        V2_RedundancySuperstructureStates.BARGE, // alg
                        V2_RedundancySuperstructureStates.PROCESSOR, // alg
                        V2_RedundancySuperstructureStates.STOW_UP), // alg
                V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3,
                    List.of(
                        V2_RedundancySuperstructureStates.INTAKE_REEF_L3,
                        V2_RedundancySuperstructureStates.DROP_REEF_L3,
                        V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
                        V2_RedundancySuperstructureStates.BARGE,
                        V2_RedundancySuperstructureStates.PROCESSOR,
                        V2_RedundancySuperstructureStates.STOW_UP));
    reefMap.forEach(
        (from, targets) -> {
          for (V2_RedundancySuperstructureStates to : targets) {
            // forward edge can vary
            if (List.of(
                    V2_RedundancySuperstructureStates.INTAKE_REEF_L2,
                    V2_RedundancySuperstructureStates.DROP_REEF_L2,
                    V2_RedundancySuperstructureStates.INTAKE_REEF_L3,
                    V2_RedundancySuperstructureStates.DROP_REEF_L3)
                .contains(to)) {
              NoAlgaeEdges.add(new Edge(from, to));
            } else if (List.of(
                    V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
                    V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3)
                .contains(to)) {
              NoneEdges.add(new Edge(from, to));
            } else {
              AlgaeEdges.add(new Edge(from, to));
            }
            // reverse edge uses NONE
            NoneEdges.add(new Edge(to, from));
          }
        });

    // BARGE and PROCESSOR transitions (one‑way, no algae)
    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.PROCESSOR,
            V2_RedundancySuperstructureStates.SCORE_BARGE)) {
      AlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.BARGE, dest));
    }
    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3)) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.BARGE, dest));
    }

    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.SCORE_BARGE,
            V2_RedundancySuperstructureStates.BARGE));

    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.BARGE,
            V2_RedundancySuperstructureStates.SCORE_PROCESSOR)) {
      AlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.PROCESSOR, dest));
    }
    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3)) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.PROCESSOR, dest));
    }
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.SCORE_PROCESSOR,
            V2_RedundancySuperstructureStates.PROCESSOR));

    // FLOOR_INTAKE, REEF_INTAKE, REEF_DROP transitions (one‑way, no algae)
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_FLOOR,
            V2_RedundancySuperstructureStates.FLOOR_ACQUISITION));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_REEF_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_REEF_L3,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.DROP_REEF_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.DROP_REEF_L3,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3));

    // START → STOW_DOWN (one‑way, no algae)
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.START, V2_RedundancySuperstructureStates.STOW_DOWN));

    // Stow Down transitions
    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR,
            V2_RedundancySuperstructureStates.FLOOR_ACQUISITION)) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.STOW_DOWN, dest));
    }

    // STOW_DOWN <-> CLIMB
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_DOWN, V2_RedundancySuperstructureStates.CLIMB));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.CLIMB, V2_RedundancySuperstructureStates.STOW_DOWN));

    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.FUNNEL_CLOSE_WITH_STOW_DOWN));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.FUNNEL_CLOSE_WITH_STOW_DOWN,
            V2_RedundancySuperstructureStates.STOW_DOWN));

    AlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_UP,
            V2_RedundancySuperstructureStates.FUNNEL_CLOSE_WITH_STOW_UP));
    AlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.FUNNEL_CLOSE_WITH_STOW_UP,
            V2_RedundancySuperstructureStates.STOW_UP));
  }

  private static void addEdges(
      Graph<V2_RedundancySuperstructureStates, EdgeCommand> graph,
      List<Edge> edges,
      AlgaeEdge type,
      V2_RedundancyElevator elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
    for (Edge edge : edges) {
      graph.addEdge(
          edge.from(),
          edge.to(),
          EdgeCommand.builder()
              .command(
                  getEdgeCommand(edge.from(), edge.to(), elevator, manipulator, funnel, intake))
              .algaeEdgeType(type)
              .build());
    }
  }

  public static void addEdges(
      Graph<V2_RedundancySuperstructureStates, EdgeCommand> graph,
      V2_RedundancyElevator elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
    createEdges();
    addEdges(graph, NoneEdges, AlgaeEdge.NONE, elevator, manipulator, funnel, intake);
    addEdges(graph, NoAlgaeEdges, AlgaeEdge.NO_ALGAE, elevator, manipulator, funnel, intake);
    addEdges(graph, AlgaeEdges, AlgaeEdge.ALGAE, elevator, manipulator, funnel, intake);
  }
}
