package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.funnel.Funnel;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntakeConstants.IntakeState;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import frc.robot.util.AllianceFlipUtil;

public class NewCompositeCommands {
  public static final class SharedCommands {
    public static final Command resetHeading(Drive drive) {
      return Commands.runOnce(
              () -> {
                RobotState.resetRobotPose(
                    new Pose2d(
                        RobotState.getRobotPoseField().getTranslation(),
                        AllianceFlipUtil.apply(new Rotation2d())));
              })
          .ignoringDisable(true);
    }

    public static final Command climb(
        Elevator elevator, Funnel funnel, Climber climber, Drive drive) {
      return Commands.sequence(
          elevator.setPosition(ReefHeight.STOW),
          Commands.waitSeconds(0.02),
          Commands.waitUntil(elevator::atGoal),
          funnel.setClapDaddyGoal(FunnelState.CLIMB),
          Commands.parallel(
              climber.releaseClimber(),
              Commands.waitSeconds(ClimberConstants.WAIT_AFTER_RELEASE_SECONDS)),
          Commands.waitUntil(climber::climberReady),
          Commands.deadline(climber.winchClimber(), Commands.run(drive::stop)));
    }

    public static final Command setStaticReefHeight(ReefHeight height) {
      return Commands.runOnce(() -> RobotState.setReefHeight(height));
    }

    public static final Command setDynamicReefHeight(ReefHeight height, Elevator elevator) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setReefHeight(height)), elevator.setPosition());
    }

    public static final class V1_StackUpCompositeCommands {
      public static final Command intakeCoral(
          Elevator elevator, Funnel funnel, V1_StackUpManipulator manipulator) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                elevator.setPosition(ReefHeight.CORAL_INTAKE),
                Commands.waitUntil(elevator::atGoal),
                Commands.race(
                    manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())))
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command intakeCoralOverride(
          Elevator elevator, Funnel funnel, V1_StackUpManipulator manipulator) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                elevator.setPosition(ReefHeight.CORAL_INTAKE),
                Commands.waitUntil(elevator::atGoal),
                Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command scoreCoral(V1_StackUpManipulator manipulator) {
        return manipulator.scoreCoral().withTimeout(0.4);
      }

      public static final Command scoreCoralSequence(
          Elevator elevator, V1_StackUpManipulator manipulator) {
        return Commands.sequence(
            elevator.setPosition(),
            Commands.waitSeconds(0.125),
            Commands.waitUntil(elevator::atGoal),
            manipulator.scoreCoral().withTimeout(0.4));
      }

      public static final Command autoScoreCoralSequence(
          Drive drive, Elevator elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
        return Commands.either(
            autoScoreL1CoralSequence(drive, elevator, manipulator, cameras),
            Commands.sequence(
                Commands.either(
                    elevator.setPosition(ReefHeight.L2),
                    Commands.none(),
                    () ->
                        RobotState.getOIData().currentReefHeight().equals(ReefHeight.L1)
                            || RobotState.getOIData().currentReefHeight().equals(ReefHeight.STOW)
                            || RobotState.getOIData()
                                .currentReefHeight()
                                .equals(ReefHeight.CORAL_INTAKE)),
                DriveCommands.autoAlignReefCoral(drive, cameras),
                scoreCoralSequence(elevator, manipulator)),
            () -> RobotState.getOIData().currentReefHeight().equals(ReefHeight.L1));
      }

      public static final Command autoScoreL1CoralSequence(
          Drive drive, Elevator elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
        return Commands.sequence(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            scoreL1Coral(drive, elevator, manipulator));
      }

      public static final Command scoreL1Coral(
          Drive drive, Elevator elevator, V1_StackUpManipulator manipulator) {
        return Commands.sequence(
            elevator.setPosition(),
            Commands.waitSeconds(0.02),
            Commands.waitUntil(elevator::atGoal),
            Commands.parallel(
                manipulator.scoreL1Coral().withTimeout(0.8),
                Commands.sequence(
                    Commands.waitSeconds(0.05),
                    Commands.either(
                        DriveCommands.inchMovement(drive, -1, 0.1),
                        DriveCommands.inchMovement(drive, 1, 0.1),
                        () -> RobotState.getOIData().currentReefPost() == ReefPose.LEFT))));
      }

      public static final Command emergencyEject(
          Elevator elevator, V1_StackUpManipulator manipulator) {
        return Commands.sequence(
            elevator.setPosition(ReefHeight.L1),
            Commands.waitSeconds(0.125),
            Commands.waitUntil(elevator::atGoal),
            manipulator.scoreCoral().withTimeout(0.4),
            elevator.setPosition(ReefHeight.STOW));
      }

      public static final Command twerk(
          Drive drive, Elevator elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
        return Commands.deferredProxy(
            () ->
                twerk(
                    drive,
                    elevator,
                    manipulator,
                    switch (RobotState.getReefAlignData().closestReefTag()) {
                      case 10, 6, 8, 21, 17, 19 -> ReefHeight.ASS_BOT;
                      case 9, 11, 7, 22, 20, 18 -> ReefHeight.ASS_TOP;
                      default -> ReefHeight.ASS_BOT;
                    },
                    cameras));
      }

      public static final Command twerk(
          Drive drive,
          Elevator elevator,
          V1_StackUpManipulator manipulator,
          ReefHeight level,
          Camera... cameras) {
        return Commands.sequence(
            DriveCommands.autoAlignReefAlgae(drive, cameras),
            elevator.setPosition(ReefHeight.L4),
            Commands.waitUntil(elevator::atGoal),
            manipulator.toggleAlgaeArm(),
            Commands.waitSeconds(0.1),
            elevator.setPosition(level),
            manipulator.removeAlgae().until(elevator::atGoal),
            manipulator.removeAlgae().withTimeout(0.35),
            manipulator.toggleAlgaeArm());
      }
    }

    public static final class V2_RedundancyCompositeCommands {
      public static final Command intakeCoralAuto(
          Elevator elevator,
          Funnel funnel,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    ReefHeight.CORAL_INTAKE,
                    ArmState.DOWN,
                    IntakeState.STOW),
                Commands.race(
                    manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())))
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command intakeCoralDriverSequence(
          Elevator elevator,
          Funnel funnel,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    ReefHeight.CORAL_INTAKE,
                    ArmState.DOWN,
                    IntakeState.STOW),
                Commands.race(
                        manipulator.intakeCoral(), funnel.intakeCoral(() -> intake.hasCoral()))
                    .until(intake::hasCoral))
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command intakeCoralOperatorSequence(
          Elevator elevator,
          Funnel funnel,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    ReefHeight.CORAL_INTAKE,
                    ArmState.DOWN,
                    IntakeState.STOW),
                Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
            .until(intake::hasCoral)
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command intakeCoralOperatorOverrideSequence(
          Elevator elevator,
          Funnel funnel,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    ReefHeight.CORAL_INTAKE,
                    ArmState.DOWN,
                    IntakeState.STOW),
                Commands.parallel(
                        manipulator.intakeCoral(), funnel.setClapDaddyGoal(FunnelState.CLOSED))
                    .until(intake::hasCoral))
            .finallyDo(
                () -> {
                  RobotState.setIntakingCoral(false);
                });
      }

      public static final Command scoreCoral(V2_RedundancyManipulator manipulator) {
        return manipulator.scoreCoral().withTimeout(0.4);
      }

      public static final Command scoreCoralSequence(
          Elevator elevator, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
        return Commands.sequence(
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                RobotState.getOIData().currentReefHeight(),
                ArmState.DOWN,
                IntakeState.STOW),
            scoreCoral(manipulator));
      }

      public static final Command scoreL1Coral(
          Drive drive,
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return Commands.sequence(
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                RobotState.getOIData().currentReefHeight(),
                ArmState.DOWN,
                IntakeState.STOW),
            Commands.parallel(
                manipulator.scoreL1Coral().withTimeout(0.8),
                Commands.sequence(
                    Commands.waitSeconds(0.05),
                    Commands.either(
                        DriveCommands.inchMovement(drive, -1, 0.1),
                        DriveCommands.inchMovement(drive, 1, 0.1),
                        () -> RobotState.getOIData().currentReefPost() == ReefPose.LEFT))));
      }

      public static final Command autoScoreL1CoralSequence(
          Drive drive,
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake,
          Camera... cameras) {
        return Commands.sequence(                Commands.waitSeconds(0.25),

            DriveCommands.autoAlignReefCoral(drive, cameras),
            scoreL1Coral(drive, elevator, manipulator, intake));
      }

      public static final Command autoScoreCoralSequence(
          Drive drive,
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake,
          Camera... cameras) {
        return Commands.either(
            autoScoreL1CoralSequence(drive, elevator, manipulator, intake, cameras),
            Commands.parallel(Commands.sequence(
                DecisionTree.moveSequence(
                        elevator,
                        manipulator,
                        intake,
                        RobotState.getOIData().currentReefHeight().equals(ReefHeight.L4) ? ReefHeight.L3 : RobotState.getOIData().currentReefHeight(),
                        ArmState.DOWN,
                        IntakeState.STOW)
                    .onlyIf(() -> elevator.getPosition().equals(ElevatorPositions.STOW)),
                DriveCommands.autoAlignReefCoral(drive, cameras)),
                scoreCoralSequence(elevator, manipulator, intake),
                Commands.sequence(
                        DecisionTree.moveSequence(
                            elevator,
                            manipulator,
                            intake,
                            ReefHeight.L4_PLUS,
                            ArmState.DOWN,
                            IntakeState.STOW),
                        manipulator.scoreCoral().withTimeout(0.5))
                    .onlyIf(() -> elevator.getPosition().equals(ElevatorPositions.L4))),
            () -> RobotState.getOIData().currentReefHeight().equals(ReefHeight.L1));
      }

      public static final Command intakeAlgaeFromReefSequence(
          Drive drive,
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake,
          ReefHeight level,
          Camera... cameras) {
        return Commands.sequence(
            DriveCommands.autoAlignReefAlgae(drive, cameras),
            Commands.deadline(
                Commands.sequence(
                    DecisionTree.moveSequence(
                        elevator,
                        manipulator,
                        intake,
                        level,
                        ArmState.REEF_INTAKE,
                        IntakeState.STOW),
                    Commands.waitSeconds(1.0),
                    Commands.runEnd(
                            () -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)),
                            () -> drive.stop())
                        .withTimeout(0.5)),
                manipulator.intakeReefAlgae()),
            Commands.either(
                DecisionTree.moveSequence(
                    elevator, manipulator, intake, ReefHeight.STOW, ArmState.UP, IntakeState.STOW),
                DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    ReefHeight.STOW,
                    ArmState.DOWN,
                    IntakeState.STOW),
                RobotState::isHasAlgae));
      }

      public static final Command scoreAlgae(
          Elevator elevator, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
        return Commands.sequence(
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                ReefHeight.ALGAE_SCORE,
                ArmState.UP,
                IntakeState.STOW),
            manipulator.scoreAlgae());
      }

      public static final Command netHeight(
          Elevator elevator, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
        return DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                ReefHeight.ALGAE_SCORE,
                ArmState.PRE_SCORE,
                IntakeState.STOW)
            .until(() -> elevator.atGoal() && manipulator.algaeArmAtGoal() && intake.atGoal());
      }

      public static final Command autoScoreAlgae(
          Drive drive,
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return Commands.sequence(
            DriveCommands.autoAlignBargeAlgae(drive),
            netHeight(elevator, manipulator, intake),
            scoreAlgae(elevator, manipulator, intake));
      }

      public static final Command scoreProcessor(
          Elevator elevator, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
        return DecisionTree.moveSequence(
            elevator, manipulator, intake, ReefHeight.STOW, ArmState.PROCESSOR, IntakeState.STOW);
      }

      public static final Command floorIntakeSequence(
          V2_RedundancyManipulator manipulator, Elevator elevator, V2_RedundancyIntake intake) {
        return Commands.sequence(
                Commands.sequence(
                        DecisionTree.moveSequence(
                            elevator,
                            manipulator,
                            intake,
                            ReefHeight.ALGAE_FLOOR_INTAKE,
                            ArmState.FLOOR_INTAKE,
                            IntakeState.INTAKE),
                        Commands.parallel(intake.intakeAlgae(), manipulator.intakeFloorAlgae()))
                    .until(() -> RobotState.isHasAlgae()))
            .finallyDo(
                () ->
                    Commands.either(
                            DecisionTree.moveSequence(
                                elevator,
                                manipulator,
                                intake,
                                ReefHeight.STOW,
                                ArmState.UP,
                                IntakeState.STOW),
                            DecisionTree.moveSequence(
                                elevator,
                                manipulator,
                                intake,
                                ReefHeight.STOW,
                                ArmState.DOWN,
                                IntakeState.STOW),
                            RobotState::isHasAlgae)
                        .schedule());
      }
    }
  }
}
