package frc.robot.subsystems.v3_Epsilon.superstructure;

// Adjust the package path as needed
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructureActions.SubsystemActions;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructurePoses.SubsystemPoses;
// Ensure this is the

// correct package
// path

public enum V3_SuperstructureStates {
  START("START", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_DOWN("STOW_DOWN", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_UP(
      "STOW_UP",
      new SubsystemPoses(
          ReefState.STOW,
          V3_EpsilonManipulatorConstants.PivotState.STOW_UP,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  HANDOFF(
      "HANDOFF",
      new SubsystemPoses(
          ReefState.STOW,
          V3_EpsilonManipulatorConstants.PivotState.HANDOFF,
          V3_EpsilonIntakeConstants.IntakeState.HANDOFF),
      SubsystemActions.empty()),

  GROUND_INTAKE(
      "GROUND_INTAKE",
      new SubsystemPoses(
          ReefState.STOW,
          V3_EpsilonManipulatorConstants.PivotState.FLOOR_INTAKE,
          V3_EpsilonIntakeConstants.IntakeState.INTAKE_CORAL),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP,
          V3_EpsilonIntakeConstants.IntakeRollerStates.CORAL_INTAKE)),

  // Coral Prep States
  L1_PREP(
      "L1",
      new SubsystemPoses(
          ReefState.L1,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.L1),
      SubsystemActions.empty()),

  L2_PREP(
      "L2",
      new SubsystemPoses(
          ReefState.L2,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  L3_PREP(
      "L3",
      new SubsystemPoses(
          ReefState.L3,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  L4_PREP(
      "L4",
      new SubsystemPoses(
          ReefState.L4,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  // Coral Score States
  L1_SCORE(
      "L1_SCORE",
      new SubsystemPoses(
          ReefState.L1,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.L1),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP,
          V3_EpsilonIntakeConstants.IntakeRollerStates.SCORE_CORAL)),

  L2_SCORE(
      "L2_SCORE",
      new SubsystemPoses(
          ReefState.L2,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_CORAL,
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  L3_SCORE(
      "L3_SCORE",
      new SubsystemPoses(
          ReefState.L3,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_CORAL,
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  L4_SCORE(
      "L4_SCORE",
      new SubsystemPoses(
          ReefState.L4,
          V3_EpsilonManipulatorConstants.PivotState
              .PRE_SCORE, // Determine whether PRE_SCORE is accurate for all scoring states
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_CORAL,
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  // Algae Prep States
  L2_ALGAE_PREP(
      "L2_ALGAE_PREP",
      new SubsystemPoses(
          ReefState.L2,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  L3_ALGAE_PREP(
      "L3_ALGAE_PREP",
      new SubsystemPoses(
          ReefState.L3,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  // Algae Scoring States
  L2_ALGAE_INTAKE(
      "L2_ALGAE_INTAKE",
      new SubsystemPoses(
          ReefState.L2,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_ALGAE,
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  L3_ALGAE_INTAKE(
      "L3_ALGAE_INTAKE",
      new SubsystemPoses(
          ReefState.L3,
          V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_ALGAE,
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  // Miscelaneous States (Barge + Processor Prep and Score States)
  BARGE_PREP(
      "BARGE_PREP",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE,
          V3_EpsilonManipulatorConstants.PivotState
              .REEF_INTAKE, // Assuming REEF_INTAKE and BARGE_PREP have same rotation values
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  PROCESSOR_PREP(
      "PROCESSOR_PREP",
      new SubsystemPoses(
          ReefState.STOW,
          V3_EpsilonManipulatorConstants.PivotState.PROCESSOR,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  BARGE_SCORE(
      "BARGE_SCORE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE,
          V3_EpsilonManipulatorConstants.PivotState.REEF_INTAKE,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates
              .SCORE_ALGAE, // Assuming you score in the barge by stowing intake and moving
          // manipulator to desired position
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  PROCESSOR_SCORE(
      "PROCESSOR_SCORE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE,
          V3_EpsilonManipulatorConstants.PivotState.PROCESSOR,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.SCORE_ALGAE,
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP)),

  INTERMIDIATE_WAIT_FOR_ELEVATOR(
      "INTERMIDIATE_WAIT_FOR_ELEVATOR",
      new SubsystemPoses(
          ReefState.STOW,
          V3_EpsilonManipulatorConstants.PivotState.STOW_UP,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  // Transition States
  L2_TRANSITION(
      "L2_TRANSITION",
      new SubsystemPoses(
          ReefState.L2,
          V3_EpsilonManipulatorConstants.PivotState.TRANSITION,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  L3_TRANSITION(
      "L3_TRANSITION",
      new SubsystemPoses(
          ReefState.L3,
          V3_EpsilonManipulatorConstants.PivotState.TRANSITION,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  L4_TRANSITION(
      "L4_TRANSITION",
      new SubsystemPoses(
          ReefState.L4,
          V3_EpsilonManipulatorConstants.PivotState.TRANSITION,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  CLIMB(
      "CLIMB",
      new SubsystemPoses(
          ReefState.STOW,
          V3_EpsilonManipulatorConstants.PivotState.STOW_UP,
          V3_EpsilonIntakeConstants.IntakeState.STOW),
      SubsystemActions.empty()),

  OVERRIDE("OVERRIDE", new SubsystemPoses(), SubsystemActions.empty());

  // Readable name for state
  private final String name;

  // Target positions for all subsystems in this state
  private final SubsystemPoses subsystemPoses;

  // Actions to perform for all subsystems in this state
  private final SubsystemActions subsystemActions;

  /**
   * Constructor for V3_SuperstructureStates.
   *
   * @param name The name of the state.
   * @param pose The subsystem poses for this state.
   * @param action The subsystem actions for this state.
   */
  V3_SuperstructureStates(String name, SubsystemPoses pose, SubsystemActions action) {
    this.name = name;
    this.subsystemPoses = pose;
    this.subsystemActions = action;
  }

  /**
   * Constructor for V3_SuperstructureStates with empty actions.
   *
   * @param name The name of the state.
   * @param pose The subsystem poses for this state.
   */
  public V3_SuperstructurePoses getPose() {
    return new V3_SuperstructurePoses(name, subsystemPoses);
  }

  /**
   * Returns the actions associated with this superstructure state.
   *
   * @return The actions for this state.
   */
  public V3_SuperstructureActions getAction() {
    return new V3_SuperstructureActions(name, subsystemActions);
  }

  /**
   * Returns the name of the superstructure state.
   *
   * @return The name of the state.
   */
  public String getName() {
    return name;
  }
}
