package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates.SuperstructureStates;
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

  public record Edge(SuperstructureStates from, SuperstructureStates to) {
    public Edge(SuperstructureStates from, SuperstructureStates to) {
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
      SuperstructureStates from,
      SuperstructureStates to,
      V2_RedundancyElevator elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
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

    return pose.asCommand(
        elevator, manipulator, funnel, intake); // does all subsystem poses in paralell
  }

  private static void createEdges() {

    // CORAL-RELATED STATES
    List<SuperstructureStates> coralLevels =
        List.of(
            SuperstructureStates.L1,
            SuperstructureStates.L2,
            SuperstructureStates.L3,
            SuperstructureStates.L4);

    // stow_down <-> each coral level (bidirectional, no algae)
    for (SuperstructureStates level : coralLevels) {
      NoAlgaeEdges.add(new Edge(SuperstructureStates.STOW_DOWN, level));
      NoAlgaeEdges.add(new Edge(level, SuperstructureStates.STOW_DOWN));
    }

    // every coral level <-> every other coral level (no algae)
    for (SuperstructureStates from : coralLevels) {
      for (SuperstructureStates to : coralLevels) {
        if (from != to) {
          NoneEdges.add(new Edge(from, to));
        }
      }
    }

    // Misellaneous L4+ transitions
    NoneEdges.add(new Edge(SuperstructureStates.L4_PLUS, SuperstructureStates.L4));
    NoneEdges.add(new Edge(SuperstructureStates.L4, SuperstructureStates.L4_PLUS));
    NoAlgaeEdges.add(new Edge(SuperstructureStates.L4_PLUS, SuperstructureStates.STOW_DOWN));
    NoneEdges.add(
        new Edge(SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM, SuperstructureStates.L4_PLUS));

    // each “level” → its scoring state (one‑way, no algae)
    Map<SuperstructureStates, SuperstructureStates> coralScoreMap =
        Map.of(
            SuperstructureStates.L1, SuperstructureStates.SCORE_L1,
            SuperstructureStates.L2, SuperstructureStates.SCORE_L2,
            SuperstructureStates.L3, SuperstructureStates.SCORE_L3,
            SuperstructureStates.L4, SuperstructureStates.SCORE_L4,
            SuperstructureStates.L4_PLUS, SuperstructureStates.SCORE_L4_PLUS);
    coralScoreMap.forEach(
        (level, score) -> {
          NoAlgaeEdges.add(new Edge(score, level));
          NoAlgaeEdges.add(new Edge(level, score));
        });

    // each coral level → FLOOR_ACQUISITION and → INTERMEDIATE_WAIT_FOR_ELEVATOR (one‑way)
    for (SuperstructureStates level : List.of(SuperstructureStates.L1, SuperstructureStates.L2)) {
      for (SuperstructureStates target :
          List.of(
              SuperstructureStates.FLOOR_ACQUISITION,
              SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR)) {
        NoneEdges.add(new Edge(level, target));
      }
    }

    for (SuperstructureStates level : List.of(SuperstructureStates.L3, SuperstructureStates.L4)) {
      for (SuperstructureStates target :
          List.of(
              SuperstructureStates.REEF_ACQUISITION_L2,
              SuperstructureStates.REEF_ACQUISITION_L3,
              SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM)) {
        NoneEdges.add(new Edge(level, target));
      }
    }

    // INTAKE_CORAL ↔ STOW_DOWN (bidirectional, no algae)
    NoAlgaeEdges.add(new Edge(SuperstructureStates.STOW_DOWN, SuperstructureStates.INTAKE));
    NoAlgaeEdges.add(new Edge(SuperstructureStates.INTAKE, SuperstructureStates.STOW_DOWN));

    // INTERMEDIATE_WAIT_FOR_ELEVATOR TRANSITIONS (all one‑way, no algae)
    List<SuperstructureStates> iveDestinations =
        List.of(
            SuperstructureStates.STOW_UP,
            SuperstructureStates.REEF_ACQUISITION_L2,
            SuperstructureStates.REEF_ACQUISITION_L3,
            SuperstructureStates.BARGE,
            SuperstructureStates.PROCESSOR);
    for (SuperstructureStates dest : iveDestinations) {
      AlgaeEdges.add(new Edge(SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR, dest));
    }

    // INTERMEDIATE_WAIT_FOR_ARM TRANSITIONS (one‑way, no algae)
    List<SuperstructureStates> iwaDestinations =
        List.of(
            SuperstructureStates.STOW_DOWN,
            SuperstructureStates.L1,
            SuperstructureStates.L2,
            SuperstructureStates.FLOOR_ACQUISITION);
    for (SuperstructureStates dest : iwaDestinations) {
      NoAlgaeEdges.add(new Edge(SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM, dest));
    }

    // STOW_UP → multiple targets (two, algae)
    List<SuperstructureStates> stowUpDestinations =
        List.of(
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            SuperstructureStates.BARGE,
            SuperstructureStates.PROCESSOR);
    for (SuperstructureStates dest : stowUpDestinations) {
      AlgaeEdges.add(new Edge(SuperstructureStates.STOW_UP, dest));
      AlgaeEdges.add(new Edge(dest, SuperstructureStates.STOW_UP));
    }

    // FLOOR_ACQUISITION → multiple targets (one‑way, no algae)
    List<SuperstructureStates> floorAcqDest =
        List.of(SuperstructureStates.STOW_DOWN, SuperstructureStates.INTAKE_FLOOR);
    for (SuperstructureStates dest : floorAcqDest) {
      NoAlgaeEdges.add(new Edge(SuperstructureStates.FLOOR_ACQUISITION, dest));
    }
    NoneEdges.add(
        new Edge(
            SuperstructureStates.FLOOR_ACQUISITION,
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR));

    // REEF-RELATED ACQUISITION STATES (using algaeMap style)
    // Define “from → to” for algae acquisition (one‑way with ALGAE), then add reverse with
    // NO_ALGAE
    Map<SuperstructureStates, List<SuperstructureStates>>
        reefMap = // Algae states here are probably wrong
        Map.of(
                SuperstructureStates.REEF_ACQUISITION_L2,
                    List.of(
                        SuperstructureStates.INTAKE_REEF_L2, // no alg
                        SuperstructureStates.DROP_REEF_L2, // no alg
                        SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM, // none
                        SuperstructureStates.REEF_ACQUISITION_L3, // none
                        SuperstructureStates.BARGE, // alg
                        SuperstructureStates.PROCESSOR, // alg
                        SuperstructureStates.STOW_UP), // alg
                SuperstructureStates.REEF_ACQUISITION_L3,
                    List.of(
                        SuperstructureStates.INTAKE_REEF_L3,
                        SuperstructureStates.DROP_REEF_L3,
                        SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
                        SuperstructureStates.BARGE,
                        SuperstructureStates.PROCESSOR,
                        SuperstructureStates.STOW_UP));
    reefMap.forEach(
        (from, targets) -> {
          for (SuperstructureStates to : targets) {
            // forward edge can vary
            if (List.of(
                    SuperstructureStates.INTAKE_REEF_L2,
                    SuperstructureStates.DROP_REEF_L2,
                    SuperstructureStates.INTAKE_REEF_L3,
                    SuperstructureStates.DROP_REEF_L3)
                .contains(to)) {
              NoAlgaeEdges.add(new Edge(from, to));
            } else if (List.of(
                    SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
                    SuperstructureStates.REEF_ACQUISITION_L3)
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
    for (SuperstructureStates dest :
        List.of(SuperstructureStates.PROCESSOR, SuperstructureStates.SCORE_BARGE)) {
      AlgaeEdges.add(new Edge(SuperstructureStates.BARGE, dest));
    }
    for (SuperstructureStates dest :
        List.of(
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            SuperstructureStates.REEF_ACQUISITION_L2,
            SuperstructureStates.REEF_ACQUISITION_L3)) {
      NoAlgaeEdges.add(new Edge(SuperstructureStates.BARGE, dest));
    }

    NoneEdges.add(new Edge(SuperstructureStates.SCORE_BARGE, SuperstructureStates.BARGE));

    for (SuperstructureStates dest :
        List.of(SuperstructureStates.BARGE, SuperstructureStates.SCORE_PROCESSOR)) {
      AlgaeEdges.add(new Edge(SuperstructureStates.PROCESSOR, dest));
    }
    for (SuperstructureStates dest :
        List.of(
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            SuperstructureStates.REEF_ACQUISITION_L2,
            SuperstructureStates.REEF_ACQUISITION_L3)) {
      NoAlgaeEdges.add(new Edge(SuperstructureStates.PROCESSOR, dest));
    }
    NoneEdges.add(new Edge(SuperstructureStates.SCORE_PROCESSOR, SuperstructureStates.PROCESSOR));

    // FLOOR_INTAKE, REEF_INTAKE, REEF_DROP transitions (one‑way, no algae)
    NoneEdges.add(
        new Edge(SuperstructureStates.INTAKE_FLOOR, SuperstructureStates.FLOOR_ACQUISITION));
    NoneEdges.add(
        new Edge(SuperstructureStates.INTAKE_REEF_L2, SuperstructureStates.REEF_ACQUISITION_L2));
    NoneEdges.add(
        new Edge(SuperstructureStates.INTAKE_REEF_L3, SuperstructureStates.REEF_ACQUISITION_L3));
    NoneEdges.add(
        new Edge(SuperstructureStates.DROP_REEF_L2, SuperstructureStates.REEF_ACQUISITION_L2));
    NoneEdges.add(
        new Edge(SuperstructureStates.DROP_REEF_L3, SuperstructureStates.REEF_ACQUISITION_L3));

    // START → STOW_DOWN (one‑way, no algae)
    NoneEdges.add(new Edge(SuperstructureStates.START, SuperstructureStates.STOW_DOWN));

    // Stow Down transitions
    for (SuperstructureStates dest :
        List.of(
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR,
            SuperstructureStates.FLOOR_ACQUISITION)) {
      NoAlgaeEdges.add(new Edge(SuperstructureStates.STOW_DOWN, dest));
    }

    // STOW_DOWN <-> CLIMB
    NoAlgaeEdges.add(new Edge(SuperstructureStates.STOW_DOWN, SuperstructureStates.CLIMB));
    NoAlgaeEdges.add(new Edge(SuperstructureStates.CLIMB, SuperstructureStates.STOW_DOWN));

    NoAlgaeEdges.add(
        new Edge(SuperstructureStates.STOW_DOWN, SuperstructureStates.FUNNEL_CLOSE_WITH_STOW_DOWN));
    NoAlgaeEdges.add(
        new Edge(SuperstructureStates.FUNNEL_CLOSE_WITH_STOW_DOWN, SuperstructureStates.STOW_DOWN));

    AlgaeEdges.add(
        new Edge(SuperstructureStates.STOW_UP, SuperstructureStates.FUNNEL_CLOSE_WITH_STOW_UP));
    AlgaeEdges.add(
        new Edge(SuperstructureStates.FUNNEL_CLOSE_WITH_STOW_UP, SuperstructureStates.STOW_UP));
  }

  private static void addEdges(
      Graph<SuperstructureStates, EdgeCommand> graph,
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
      Graph<SuperstructureStates, EdgeCommand> graph,
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
