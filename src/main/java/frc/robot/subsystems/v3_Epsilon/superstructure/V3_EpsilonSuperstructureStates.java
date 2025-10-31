package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureAction.SubsystemActions;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorArmState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorRollerState;
import lombok.Getter;

public enum V3_EpsilonSuperstructureStates {
  START("START", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_DOWN(
      "STOW_DOWN",
      new SubsystemPoses(ReefState.STOW, ManipulatorArmState.VERTICAL_UP, IntakePivotState.STOW),
      SubsystemActions.empty()),
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
      new SubsystemPoses(
          ReefState.ALGAE_FLOOR_INTAKE,
          ManipulatorArmState.ALGAE_INTAKE_FLOOR,
          IntakePivotState.INTAKE_ALGAE),
      new SubsystemActions(ManipulatorRollerState.ALGAE_INTAKE, IntakeRollerState.STOP)),
  GROUND_AQUISITION_ALGAE(
      "GROUND_AQUISITION_ALGAE",
      new SubsystemPoses(
          ReefState.ALGAE_FLOOR_INTAKE,
          ManipulatorArmState.ALGAE_INTAKE_FLOOR,
          IntakePivotState.INTAKE_ALGAE),
      SubsystemActions.empty()),

  GROUND_INTAKE_CORAL(
      "GROUND_INTAKE_CORAL",
      new SubsystemPoses(
          ReefState.STOW, ManipulatorArmState.CORAL_INTAKE_FLOOR, IntakePivotState.INTAKE_CORAL),
      new SubsystemActions(ManipulatorRollerState.CORAL_INTAKE, IntakeRollerState.STOP)),

  GROUND_INTAKE(
      "GROUND_INTAKE",
      new SubsystemPoses(
          ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.INTAKE_CORAL),
      new SubsystemActions(ManipulatorRollerState.CORAL_INTAKE, IntakeRollerState.CORAL_INTAKE)),

  L1(
      "L1",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.L1),
      new SubsystemActions(ManipulatorRollerState.STOP, IntakeRollerState.STOP)),
  L1_SCORE(
      "L1_SCORE",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.L1),
      new SubsystemActions(ManipulatorRollerState.STOP, IntakeRollerState.SCORE_CORAL)),

  L2(
      "L2",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.TRANSITION, IntakePivotState.HANDOFF),
      SubsystemActions.empty()),
  L2_SCORE(
      "L2_SCORE",
      new SubsystemPoses(ReefState.L2, ManipulatorArmState.FLIPPED_SCORE, IntakePivotState.HANDOFF),
      new SubsystemActions(ManipulatorRollerState.SCORE_CORAL, IntakeRollerState.STOP)),

  L3(
      "L3",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.TRANSITION, IntakePivotState.HANDOFF),
      SubsystemActions.empty()),
  L3_SCORE(
      "L3_SCORE",
      new SubsystemPoses(ReefState.L3, ManipulatorArmState.SCORE, IntakePivotState.HANDOFF),
      new SubsystemActions(ManipulatorRollerState.SCORE_CORAL, IntakeRollerState.STOP)),

  L4(
      "L4",
      new SubsystemPoses(ReefState.L4, ManipulatorArmState.TRANSITION, IntakePivotState.HANDOFF),
      SubsystemActions.empty()),
  L4_SCORE(
      "L4_SCORE",
      new SubsystemPoses(ReefState.L4, ManipulatorArmState.SCORE_L4, IntakePivotState.HANDOFF),
      new SubsystemActions(ManipulatorRollerState.L4_SCORE, IntakeRollerState.STOP)),

  HANDOFF(
      "HANDOFF",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.HANDOFF),
      new SubsystemActions(ManipulatorRollerState.STOP, IntakeRollerState.CENTERING)),

  HANDOFF_SPIN(
      "HANDOFF",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.HANDOFF, IntakePivotState.HANDOFF),
      new SubsystemActions(ManipulatorRollerState.CORAL_INTAKE, IntakeRollerState.OUTTAKE)),

  L2_ALGAE(
      "L2_ALGAE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      SubsystemActions.empty()),
  L2_ALGAE_DROP(
      "L2_ALGAE_DROP",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP)),
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
  L3_ALGAE_DROP(
      "L3_ALGAE_DROP",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP)),
  L3_ALGAE_INTAKE(
      "L3_ALGAE_INTAKE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP, ManipulatorArmState.REEF_INTAKE, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.ALGAE_INTAKE, IntakeRollerState.STOP)),

  PROCESSOR(
      "PROCESSOR",
      new SubsystemPoses(ReefState.STOW, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      SubsystemActions.empty()),
  POST_PROCESSOR(
      "POST_PROCESSOR",
      new SubsystemPoses(
          ReefState.POST_PROCESSOR, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      SubsystemActions.empty()),
  PROCESSOR_SCORE(
      "PROCESSOR_SCORE",
      new SubsystemPoses(ReefState.STOW, ManipulatorArmState.PROCESSOR, IntakePivotState.STOW),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP)),

  BARGE(
      "BARGE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, ManipulatorArmState.VERTICAL_UP, IntakePivotState.HANDOFF),
      SubsystemActions.empty()),
  BARGE_SCORE(
      "BARGE_SCORE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, ManipulatorArmState.ALGAESCORE, IntakePivotState.HANDOFF),
      new SubsystemActions(ManipulatorRollerState.SCORE_ALGAE, IntakeRollerState.STOP),
      Rotation2d.fromDegrees(15)),
  FLIP_DOWN(
      "FLIP_DOWN",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.FLIP_ANGLE, IntakePivotState.STOW),
      SubsystemActions.empty()),
  FLIP_UP(
      "FLIP_UP",
      new SubsystemPoses(ReefState.HANDOFF, ManipulatorArmState.FLIP_ANGLE, IntakePivotState.STOW),
      SubsystemActions.empty()),
  INVERSE_FLIP_UP(
      "INVERSE_FLIP_UP",
      new SubsystemPoses(
          ReefState.HANDOFF, ManipulatorArmState.INVERSE_FLIP_ANGLE, IntakePivotState.STOW),
      SubsystemActions.empty());

  @Getter private final V3_EpsilonSuperstructurePose pose;
  @Getter private final V3_EpsilonSuperstructureAction action;

  /**
   * Constructor for V3_SuperstructureStates.
   *
   * @param name The name of the state.
   * @param pose The subsystem poses for this state.
   * @param action The subsystem actions for this state.
   */
  V3_EpsilonSuperstructureStates(String name, SubsystemPoses pose, SubsystemActions action) {
    this.pose = new V3_EpsilonSuperstructurePose(name, pose);
    this.action = new V3_EpsilonSuperstructureAction(name, action);
  }

  V3_EpsilonSuperstructureStates(
      String name, SubsystemPoses pose, SubsystemActions action, Double elevatorTolerance) {
    this.pose = new V3_EpsilonSuperstructurePose(name, pose, elevatorTolerance);
    this.action = new V3_EpsilonSuperstructureAction(name, action);
  }

  V3_EpsilonSuperstructureStates(
      String name, SubsystemPoses pose, SubsystemActions action, Rotation2d armTolerance) {
    this.pose = new V3_EpsilonSuperstructurePose(name, pose, armTolerance);
    this.action = new V3_EpsilonSuperstructureAction(name, action);
  }
}
