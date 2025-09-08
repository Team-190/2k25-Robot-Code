package frc.robot.subsystems.v3_Epsilon.superstructure;

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
      new SubsystemPoses(
          ReefState.HIGH_STOW, ManipulatorArmState.VERTICAL_UP, IntakePivotState.STOW),
      SubsystemActions.empty()),
  OVERRIDE("OVERRIDE", new SubsystemPoses(), SubsystemActions.empty()),

  GROUND_ACQUISITION(
      "GROUND_ACQUISITION",
      new SubsystemPoses(
          ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.INTAKE_CORAL),
      SubsystemActions.empty()),
  GROUND_INTAKE_ALGAE(
      "GROUND_INTAKE_ALGAE",
      new SubsystemPoses(ReefState.HIGH_STOW, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.ALGAE_INTAKE, IntakeRollerState.STOP)),
  GROUND_AQUISITION_ALGAE(
      "GROUND_AQUISITION_ALGAE",
      new SubsystemPoses(ReefState.HIGH_STOW, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      SubsystemActions.empty()),

  GROUND_INTAKE(
      "GROUND_INTAKE",
      new SubsystemPoses(
          ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.INTAKE_CORAL),
      new SubsystemActions(ManipulatorRollerState.STOP, IntakeRollerState.CORAL_INTAKE)),

  L1(
      "L1",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.L1),
      SubsystemActions.empty()),
  L1_SCORE(
      "L1_SCORE",
      new SubsystemPoses(ReefState.L1, ManipulatorArmState.HANDOFF, IntakePivotState.L1),
      new SubsystemActions(ManipulatorRollerState.STOP, IntakeRollerState.SCORE_CORAL)),

  L2(
      "L2",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.TRANSITION, IntakePivotState.STOW),
      SubsystemActions.empty()),
  L2_SCORE(
      "L2_SCORE",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.SCORE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_CORAL, IntakeRollerState.STOP)),

  L3(
      "L3",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.TRANSITION, IntakePivotState.STOW),
      SubsystemActions.empty()),
  L3_SCORE(
      "L3_SCORE",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.SCORE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_CORAL, IntakeRollerState.STOP)),

  L4(
      "L4",
      new SubsystemPoses(ReefState.L4, ManipulatorArmState.TRANSITION, IntakePivotState.STOW),
      SubsystemActions.empty()),
  L4_SCORE(
      "L4_SCORE",
      new SubsystemPoses(ReefState.L4, ManipulatorArmState.SCORE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.L4_SCORE, IntakeRollerState.STOP)),

  HANDOFF(
      "HANDOFF",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.CORAL_INTAKE, IntakeRollerState.OUTTAKE)),

  INTERMEDIATE_WAIT_FOR_ELEVATOR(
      "INTERMEDIATE_WAIT_FOR_ELEVATOR",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.VERTICAL_UP, IntakePivotState.STOW),
      SubsystemActions.empty()),
  INTERMEDIATE_WAIT_FOR_ARM(
      "INTERMEDIATE_WAIT_FOR_ARM",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.SAFE_ANGLE, IntakePivotState.STOW),
      SubsystemActions.empty()),

  L2_ALGAE(
      "L2_ALGAE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      SubsystemActions.empty()),
  L2_ALGAE_INTAKE(
      "L2_ALGAE_INTAKE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.ALGAE_INTAKE, IntakeRollerState.STOP)),

  L3_ALGAE(
      "L3_ALGAE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      SubsystemActions.empty()),
  L3_ALGAE_INTAKE(
      "L3_ALGAE_INTAKE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.ALGAE_INTAKE, IntakeRollerState.STOP)),

  PROCESSOR(
      "PROCESSOR",
      new SubsystemPoses(ReefState.STOW, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      SubsystemActions.empty()),
  PROCESSOR_SCORE(
      "PROCESSOR_SCORE",
      new SubsystemPoses(ReefState.STOW, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP)),

  BARGE(
      "BARGE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, ManipulatorArmState.TRANSITION, IntakePivotState.STOW),
      SubsystemActions.empty()),
  BARGE_SCORE(
      "BARGE_SCORE",
      new SubsystemPoses(ReefState.ALGAE_SCORE, ManipulatorArmState.SCORE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP)),
  ;

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
