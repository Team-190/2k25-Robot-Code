package frc.robot.subsystems.v2_Redundancy.superstructure;

import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureAction.SubsystemActions;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeExtensionState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class V2_RedundancyStates {
  public static final List<SuperstructureStates> Actions =
      List.of(
          SuperstructureStates.INTAKE,
          SuperstructureStates.SCORE_L1,
          SuperstructureStates.SCORE_L2,
          SuperstructureStates.SCORE_L3,
          SuperstructureStates.SCORE_L4,
          SuperstructureStates.SCORE_L4_PLUS,
          SuperstructureStates.INTAKE_FLOOR,
          SuperstructureStates.INTAKE_REEF_L2,
          SuperstructureStates.INTAKE_REEF_L3,
          SuperstructureStates.DROP_REEF_L2,
          SuperstructureStates.DROP_REEF_L3,
          SuperstructureStates.SCORE_BARGE,
          SuperstructureStates.SCORE_PROCESSOR);

  public static final ArrayList<Edge> NoneEdges = new ArrayList<>();
  public static final ArrayList<Edge> AlgaeEdges = new ArrayList<>();
  public static final ArrayList<Edge> NoAlgaeEdges = new ArrayList<>();

  public enum SuperstructureStates {
    START("START", new SubsystemPoses(), SubsystemActions.empty()),
    STOW_DOWN("STOW DOWN", new SubsystemPoses(), SubsystemActions.empty()),
    OVERRIDE("OVERRIDE", new SubsystemPoses(), SubsystemActions.empty()),
    INTAKE(
        "INTAKE CORAL",
        new SubsystemPoses(),
        new SubsystemActions(
            ManipulatorRollerState.CORAL_INTAKE, FunnelRollerState.INTAKE, IntakeRollerState.STOP)),
    L1(
        "L1 CORAL SETPOINT",
        new SubsystemPoses(
            ReefState.L1, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        SubsystemActions.empty()),
    L2(
        "L2 CORAL SETPOINT",
        new SubsystemPoses(
            ReefState.L2, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        SubsystemActions.empty()),
    L3(
        "L3 CORAL SETPOINT",
        new SubsystemPoses(
            ReefState.L3, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        SubsystemActions.empty()),
    L4(
        "L4 CORAL SETPOINT",
        new SubsystemPoses(
            ReefState.L4, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        SubsystemActions.empty()),
    L4_PLUS(
        "L4+ CORAL SETPOINT",
        new SubsystemPoses(
            ReefState.L4_PLUS, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        SubsystemActions.empty()),
    SCORE_L1(
        "L1 CORAL SCORE",
        new SubsystemPoses(
            ReefState.L1, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.L1_SCORE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    SCORE_L2(
        "L2 CORAL SCORE",
        new SubsystemPoses(
            ReefState.L2, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.SCORE_CORAL, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    SCORE_L3(
        "L3 CORAL SCORE",
        new SubsystemPoses(
            ReefState.L3, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.SCORE_CORAL, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    SCORE_L4(
        "L4 CORAL SCORE",
        new SubsystemPoses(
            ReefState.L4, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.L4_SCORE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    SCORE_L4_PLUS(
        "L4+ CORAL SCORE",
        new SubsystemPoses(
            ReefState.L4_PLUS, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.SCORE_CORAL, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    INTERMEDIATE_WAIT_FOR_ELEVATOR(
        "WAIT FOR ELEVATOR",
        new SubsystemPoses(
            ReefState.ALGAE_MID, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        SubsystemActions.empty()),
    INTERMEDIATE_WAIT_FOR_ARM(
        "WAIT FOR ARM",
        new SubsystemPoses(
            ReefState.ALGAE_MID, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
        SubsystemActions.empty()),
    STOW_UP(
        "STOW UP",
        new SubsystemPoses(
            ReefState.STOW, ArmState.STOW_UP, IntakeExtensionState.STOW, FunnelState.OPENED),
        SubsystemActions.empty()),
    FLOOR_ACQUISITION(
        "FLOOR ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_FLOOR_INTAKE,
            ArmState.FLOOR_INTAKE,
            IntakeExtensionState.INTAKE,
            FunnelState.OPENED),
        SubsystemActions.empty()),
    REEF_ACQUISITION_L2(
        "L2 ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_BOTTOM,
            ArmState.REEF_INTAKE,
            IntakeExtensionState.STOW,
            FunnelState.OPENED),
        SubsystemActions.empty()),
    REEF_ACQUISITION_L3(
        "L3 ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_TOP,
            ArmState.REEF_INTAKE,
            IntakeExtensionState.STOW,
            FunnelState.OPENED),
        SubsystemActions.empty()),
    BARGE(
        "BARGE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_SCORE, ArmState.STOW_UP, IntakeExtensionState.STOW, FunnelState.CLOSED),
        SubsystemActions.empty()),
    PROCESSOR(
        "PROCESSOR SETPOINT",
        new SubsystemPoses(
            ReefState.STOW, ArmState.PROCESSOR, IntakeExtensionState.STOW, FunnelState.CLOSED),
        SubsystemActions.empty()),
    INTAKE_FLOOR(
        "INTAKE FLOOR",
        new SubsystemPoses(
            ReefState.ALGAE_FLOOR_INTAKE,
            ArmState.FLOOR_INTAKE,
            IntakeExtensionState.INTAKE,
            FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.ALGAE_INTAKE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    INTAKE_REEF_L2(
        "L2 ALGAE INTAKE",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_BOTTOM,
            ArmState.REEF_INTAKE,
            IntakeExtensionState.STOW,
            FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.ALGAE_INTAKE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    INTAKE_REEF_L3(
        "L3 ALGAE INTAKE",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_TOP,
            ArmState.REEF_INTAKE,
            IntakeExtensionState.STOW,
            FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.ALGAE_INTAKE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    DROP_REEF_L2(
        "DROP L2 ALGAE",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_BOTTOM,
            ArmState.REEF_INTAKE,
            IntakeExtensionState.STOW,
            FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.REMOVE_ALGAE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    DROP_REEF_L3(
        "DROP L3 ALGAE",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_TOP,
            ArmState.REEF_INTAKE,
            IntakeExtensionState.STOW,
            FunnelState.OPENED),
        new SubsystemActions(
            ManipulatorRollerState.REMOVE_ALGAE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    SCORE_BARGE(
        "SCORE BARGE",
        new SubsystemPoses(
            ReefState.ALGAE_SCORE, ArmState.STOW_UP, IntakeExtensionState.STOW, FunnelState.CLOSED),
        new SubsystemActions(
            ManipulatorRollerState.SCORE_ALGAE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    SCORE_PROCESSOR(
        "SCORE PROCESSOR",
        new SubsystemPoses(
            ReefState.STOW, ArmState.PROCESSOR, IntakeExtensionState.STOW, FunnelState.CLOSED),
        new SubsystemActions(
            ManipulatorRollerState.SCORE_ALGAE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
    CLIMB(
        "CLIMB",
        new SubsystemPoses(
            ReefState.STOW, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.CLIMB),
        SubsystemActions.empty()),
    FUNNEL_CLOSE_WITH_STOW_UP(
        "FUNNEL CLOSE WITH STOW UP",
        new SubsystemPoses(
            ReefState.STOW, ArmState.STOW_UP, IntakeExtensionState.STOW, FunnelState.CLOSED),
        SubsystemActions.empty()),
    FUNNEL_CLOSE_WITH_STOW_DOWN(
        "FUNNEL CLOSE WITH STOW DOWN",
        new SubsystemPoses(
            ReefState.STOW, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.CLOSED),
        SubsystemActions.empty());
    private final String name;
    private final SubsystemPoses subsystemPoses;
    private final SubsystemActions subsystemActions;

    private SuperstructureStates(String name, SubsystemPoses poses, SubsystemActions rollerStates) {
      this.name = name;
      this.subsystemPoses = poses;
      this.subsystemActions = rollerStates;
    }

    public V2_RedundancySuperstructurePose getPose() {
      return new V2_RedundancySuperstructurePose(name, subsystemPoses);
    }

    public V2_RedundancySuperstructureAction getAction() {
      return new V2_RedundancySuperstructureAction(name, subsystemActions);
    }

    @Override
    public String toString() {
      return name;
    }
  }

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

  public static void createEdges() {

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
}
