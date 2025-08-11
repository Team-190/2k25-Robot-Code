package frc.robot.subsystems.v3_Epsilon.superstructure;

// Adjust the package path as needed
import frc.robot.FieldConstants;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructureActions.SubsystemActions;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructurePoses.SubsystemPoses; // Ensure this is the correct package path

public enum V3_SuperstructureStates {
  START("START", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_DOWN("STOW_DOWN", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_UP(
      "STOW_UP",
      new SubsystemPoses(
          FieldConstants.Reef.ReefState.STOW,
          V3_EpsilonManipulatorConstants.PivotState.STOW_UP,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  // Coral Prep States
  L1_PREP(
      "L1",
      new SubsystemPoses(
          FieldConstants.Reef.ReefState.L1,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.L1),
      SubsystemActions.empty()),

  L2_PREP(
      "L2",
      new SubsystemPoses(
          FieldConstants.Reef.ReefState.L2,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  L3_PREP(
      "L3",
      new SubsystemPoses(
          FieldConstants.Reef.ReefState.L3,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),
  
  L4_PREP(
      "L4",
      new SubsystemPoses(
          FieldConstants.Reef.ReefState.L4,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  // Coral Score States
  L1_SCORE(
      "L1_SCORE",
      new SubsystemPoses(
          FieldConstants.Reef.ReefState.L1,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.L1),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP,
          V3_EpsilonIntakeConstants.IntakeRollerStates.SCORE_CORAL)),

  L2_SCORE(
    "L2_SCORE",
    new SubsystemPoses(
        FieldConstants.Reef.ReefState.L2,
        V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
        V3_EpsilonIntakeConstants.IntakeState.STOW),
    new SubsystemActions(
        V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_CORAL,
        V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  L3_SCORE(
    "L3_SCORE",
    new SubsystemPoses(
        FieldConstants.Reef.ReefState.L3,
        V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
        V3_EpsilonIntakeConstants.IntakeState.STOW),
    new SubsystemActions(
        V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_CORAL,
        V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  L4_SCORE(
    "L4_SCORE",
    new SubsystemPoses(
        FieldConstants.Reef.ReefState.L4,
        V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE, // Determine whether PRE_SCORE is accurate for all scoring states
        V3_EpsilonIntakeConstants.IntakeState.STOW),
    new SubsystemActions(
        V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_CORAL,
        V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)
  ),

  // Algae Prep States
  L2_ALGAE_PREP(
        "L2_ALGAE_PREP",
        new SubsystemPoses(
            FieldConstants.Reef.ReefState.L2,
            V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
            V3_EpsilonIntakeConstants.IntakeState.STOW),
        SubsystemActions.empty()),

  L3_ALGAE_PREP(
        "L3_ALGAE_PREP",
        new SubsystemPoses(
            FieldConstants.Reef.ReefState.L3,
            V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
            V3_EpsilonIntakeConstants.IntakeState.STOW),
        SubsystemActions.empty()),

  // Algae Scoring States
  L2_ALGAE_SCORE(
        "L2_ALGAE_SCORE",
        new SubsystemPoses(
            FieldConstants.Reef.ReefState.L2,
            V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
            V3_EpsilonIntakeConstants.IntakeState.STOW),
        new SubsystemActions(
            V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_ALGAE,
            V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),
  
  L3_ALGAE_SCORE(
        "L3_ALGAE_SCORE",
        new SubsystemPoses(
            FieldConstants.Reef.ReefState.L3,
            V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
            V3_EpsilonIntakeConstants.IntakeState.STOW),
        new SubsystemActions(
            V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_ALGAE,
            V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  // Miscelaneous States (Barge + Processor)
  BARGE_PREP(
        "BARGE_PREP",
        new SubsystemPoses(
            FieldConstants.Reef.ReefState.ASS_TOP,
            V3_EpsilonManipulatorConstants.PivotState.REEF_INTAKE, // Assuming REEF_INTAKE and BARGE_PREP have same rotation values
            V3_EpsilonIntakeConstants.IntakeState.STOW),
        SubsystemActions.empty()),

  PROCESSOR_PREP(
    "PROCESSOR_PREP",
    new SubsystemPoses(
        FieldConstants.Reef.ReefState.ASS_BOT,
        V3_EpsilonManipulatorConstants.PivotState.PROCESSOR,
        V3_EpsilonIntakeConstants.IntakeState.STOW),
    SubsystemActions.empty()),

  BARGE_SCORE(
    "BARGE_SCORE",
    new SubsystemPoses(
        FieldConstants.Reef.ReefState.ASS_TOP, // Assuming ASS_TOP is the max height the elevator can reach. Check later
        V3_EpsilonManipulatorConstants.PivotState.REEF_INTAKE,
        V3_EpsilonIntakeConstants.IntakeState.STOW),
    new SubsystemActions(
        V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_ALGAE, // Assuming you score in the barge by stowing intake and moving manipulator to desired position and then scoring
        V3_EpsilonIntakeConstants.IntakeRollerStates.STOP
    )),

   PROCESSOR_SCORE(
        "PROCESSOR_SCORE",
        new SubsystemPoses(
            FieldConstants.Reef.ReefState.ASS_BOT,
            V3_EpsilonManipulatorConstants.PivotState.PROCESSOR,
            V3_EpsilonIntakeConstants.IntakeState.STOW
        ),
        new SubsystemActions(
            V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_ALGAE,
            V3_EpsilonIntakeConstants.IntakeRollerStates.STOP
        ));

  private final String name;
  private final SubsystemPoses pose;
  private final SubsystemActions action;

  V3_SuperstructureStates(String name, SubsystemPoses pose, SubsystemActions action) {
    this.name = name;
    this.pose = pose;
    this.action = action;
  }
}
