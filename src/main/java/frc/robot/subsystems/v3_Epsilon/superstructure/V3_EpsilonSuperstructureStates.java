package frc.robot.subsystems.v3_Epsilon.superstructure;

// Adjust the package path as needed
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureAction.SubsystemActions;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorArmState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorRollerState;

// correct package
// path

public enum V3_EpsilonSuperstructureStates {
  START("START", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_DOWN("STOW_DOWN", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_UP(
      "STOW_UP",
      new SubsystemPoses(ReefState.STOW, ManipulatorArmState.STOW_UP, IntakePivotState.STOW),
      SubsystemActions.empty()),

  HANDOFF(
      "HANDOFF",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.HANDOFF),
      SubsystemActions.empty()),

  GROUND_INTAKE(
      "GROUND_INTAKE",
      new SubsystemPoses(
          ReefState.STOW, ManipulatorArmState.FLOOR_INTAKE, IntakePivotState.INTAKE_CORAL),
      new SubsystemActions(ManipulatorRollerState.STOP, IntakeRollerState.CORAL_INTAKE)),

  // Coral Prep States
  L1_PREP(
      "L1",
      new SubsystemPoses(ReefState.L1, ManipulatorArmState.PRE_SCORE, IntakePivotState.L1),
      SubsystemActions.empty()),

  L2_PREP(
      "L2",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      SubsystemActions.empty()),

  L3_PREP(
      "L3",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      SubsystemActions.empty()),

  L4_PREP(
      "L4",
      new SubsystemPoses(ReefState.L4, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      SubsystemActions.empty()),

  // Coral Score States
  L1_SCORE(
      "L1_SCORE",
      new SubsystemPoses(ReefState.L1, ManipulatorArmState.PRE_SCORE, IntakePivotState.L1),
      new SubsystemActions(ManipulatorRollerState.STOP, IntakeRollerState.SCORE_CORAL)),

  L2_SCORE(
      "L2_SCORE",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_CORAL, IntakeRollerState.STOP)),

  L3_SCORE(
      "L3_SCORE",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_CORAL, IntakeRollerState.STOP)),

  L4_SCORE(
      "L4_SCORE",
      new SubsystemPoses(
          ReefState.L4,
          ManipulatorArmState
              .PRE_SCORE, // Determine whether PRE_SCORE is accurate for all scoring states
          IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_CORAL, IntakeRollerState.STOP)),

  // Algae Prep States
  L2_ALGAE_PREP(
      "L2_ALGAE_PREP",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      SubsystemActions.empty()),

  L3_ALGAE_PREP(
      "L3_ALGAE_PREP",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      SubsystemActions.empty()),

  // Algae Scoring States
  L2_ALGAE_INTAKE(
      "L2_ALGAE_INTAKE",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP)),

  L3_ALGAE_INTAKE(
      "L3_ALGAE_INTAKE",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.PRE_SCORE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP)),

  // Miscelaneous States (Barge + Processor Prep and Score States)
  BARGE_PREP(
      "BARGE_PREP",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE,
          ManipulatorArmState
              .REEF_INTAKE, // Assuming REEF_INTAKE and BARGE_PREP have same rotation values
          IntakePivotState.STOW),
      SubsystemActions.empty()),

  PROCESSOR_PREP(
      "PROCESSOR_PREP",
      new SubsystemPoses(ReefState.STOW, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      SubsystemActions.empty()),

  BARGE_SCORE(
      "BARGE_SCORE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      new SubsystemActions(
          ManipulatorRollerState
              .SCORE_ALGAE, // Assuming you score in the barge by stowing intake and moving
          // manipulator to desired position
          IntakeRollerState.STOP)),

  PROCESSOR_SCORE(
      "PROCESSOR_SCORE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP)),

  INTERMIDIATE_WAIT_FOR_ELEVATOR(
      "INTERMIDIATE_WAIT_FOR_ELEVATOR",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.STOW_UP, IntakePivotState.STOW),
      SubsystemActions.empty()),

  // Transition States
  L2_TRANSITION(
      "L2_TRANSITION",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.TRANSITION, IntakePivotState.STOW),
      SubsystemActions.empty()),

  L3_TRANSITION(
      "L3_TRANSITION",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.TRANSITION, IntakePivotState.STOW),
      SubsystemActions.empty()),

  L4_TRANSITION(
      "L4_TRANSITION",
      new SubsystemPoses(ReefState.L4, ManipulatorArmState.TRANSITION, IntakePivotState.STOW),
      SubsystemActions.empty()),

  CLIMB(
      "CLIMB",
      new SubsystemPoses(ReefState.STOW, ManipulatorArmState.STOW_UP, IntakePivotState.STOW),
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
  V3_EpsilonSuperstructureStates(String name, SubsystemPoses pose, SubsystemActions action) {
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
  public V3_EpsilonSuperstructurePose getPose() {
    return new V3_EpsilonSuperstructurePose(name, subsystemPoses);
  }

  /**
   * Returns the actions associated with this superstructure state.
   *
   * @return The actions for this state.
   */
  public V3_EpsilonSuperstructureAction getAction() {
    return new V3_EpsilonSuperstructureAction(name, subsystemActions);
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
