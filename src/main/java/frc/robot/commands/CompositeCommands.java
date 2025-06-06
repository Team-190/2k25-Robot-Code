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
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CompositeCommands {
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
          elevator.setPosition(() -> ReefHeight.STOW),
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
                elevator.setPosition(() -> ReefHeight.CORAL_INTAKE),
                Commands.waitUntil(elevator::atGoal),
                Commands.race(
                    manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())))
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command intakeCoralOverride(
          Elevator elevator, Funnel funnel, V1_StackUpManipulator manipulator) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                elevator.setPosition(() -> ReefHeight.CORAL_INTAKE),
                Commands.waitUntil(elevator::atGoal),
                Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command scoreCoral(V1_StackUpManipulator manipulator) {
        return manipulator.scoreCoral().withTimeout(0.4);
      }

      public static final Command scoreCoralSequence(
          Elevator elevator, V1_StackUpManipulator manipulator, BooleanSupplier autoAligned) {
        return Commands.sequence(
            Commands.either(
                elevator.setPosition(() -> ReefHeight.L3),
                elevator.setPosition(),
                () ->
                    RobotState.getOIData().currentReefHeight().equals(ReefHeight.L4)
                        && !elevator.getPosition().equals(ElevatorPositions.L4)),
            Commands.waitUntil(() -> autoAligned.getAsBoolean()),
            elevator.setPosition(),
            Commands.waitSeconds(0.05),
            Commands.waitUntil(elevator::atGoal),
            Commands.either(
                manipulator.scoreL4Coral().withTimeout(0.4),
                manipulator.scoreCoral().withTimeout(0.15),
                () -> RobotState.getOIData().currentReefHeight().equals(ReefHeight.L4)));
      }

      public static final Command autoScoreCoralSequence(
          Drive drive, Elevator elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
        return Commands.either(
            autoScoreL1CoralSequence(drive, elevator, manipulator, cameras),
            Commands.sequence(
                Commands.either(
                    elevator.setPosition(() -> ReefHeight.L2),
                    Commands.none(),
                    () ->
                        RobotState.getOIData().currentReefHeight().equals(ReefHeight.L1)
                            || RobotState.getOIData().currentReefHeight().equals(ReefHeight.STOW)
                            || RobotState.getOIData()
                                .currentReefHeight()
                                .equals(ReefHeight.CORAL_INTAKE)),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    scoreCoralSequence(
                        elevator,
                        manipulator,
                        () -> RobotState.getReefAlignData().atCoralSetpoint())),
                elevator
                    .setPosition(() -> ReefHeight.STOW)
                    .onlyIf(
                        () ->
                            elevator.getPosition().equals(ElevatorPositions.L3)
                                || elevator.getPosition().equals(ElevatorPositions.L2))),
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
            elevator.setPosition(() -> ReefHeight.L1),
            Commands.waitSeconds(0.125),
            Commands.waitUntil(elevator::atGoal),
            manipulator.scoreCoral().withTimeout(0.4),
            elevator.setPosition(() -> ReefHeight.STOW));
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
            elevator.setPosition(() -> ReefHeight.L4),
            Commands.waitUntil(elevator::atGoal),
            manipulator.toggleAlgaeArm(),
            Commands.waitSeconds(0.1),
            elevator.setPosition(() -> level),
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
                Commands.runOnce(() -> RobotState.setHasAlgae(false)),
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                Commands.parallel(
                    DecisionTree.moveSequence(
                        elevator,
                        manipulator,
                        intake,
                        () -> ReefHeight.CORAL_INTAKE,
                        ArmState.STOW_DOWN,
                        IntakeState.STOW)),
                Commands.race(
                    manipulator.intakeCoral(() -> intake.hasCoral()),
                    funnel.intakeCoral(() -> manipulator.hasCoral())))
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command intakeCoralDriverSequence(
          Elevator elevator,
          Funnel funnel,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return Commands.sequence(
                Commands.runOnce(() -> RobotState.setHasAlgae(false)),
                Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
                DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    () -> ReefHeight.CORAL_INTAKE,
                    ArmState.STOW_DOWN,
                    IntakeState.STOW),
                Commands.race(
                        manipulator.intakeCoral(() -> intake.hasCoral()),
                        funnel.intakeCoral(() -> intake.hasCoral()))
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
                Commands.parallel(
                    DecisionTree.moveSequence(
                        elevator,
                        manipulator,
                        intake,
                        () -> ReefHeight.CORAL_INTAKE,
                        ArmState.STOW_DOWN,
                        IntakeState.STOW)),
                Commands.parallel(
                    manipulator.intakeCoral(() -> false), funnel.intakeCoral(() -> false)))
            .until(intake::hasCoral)
            .finallyDo(() -> RobotState.setIntakingCoral(false));
      }

      public static final Command intakeCoralOperatorOverrideSequence(
          Elevator elevator,
          Funnel funnel,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return funnel.setClapDaddyGoal(FunnelState.CLOSED);
      }

      public static final Command scoreCoral(V2_RedundancyManipulator manipulator) {
        return manipulator.scoreCoral().withTimeout(0.4);
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
                () -> RobotState.getOIData().currentReefHeight(),
                ArmState.STOW_DOWN,
                IntakeState.STOW),
            Commands.parallel(
                manipulator.scoreL1Coral().withTimeout(0.8),
                Commands.sequence(
                    Commands.waitSeconds(0.05),
                    Commands.either(
                        DriveCommands.inchMovement(drive, -1, 0.1),
                        DriveCommands.inchMovement(drive, 1, 0.1),
                        () -> RobotState.getOIData().currentReefPost() == ReefPose.LEFT)),
                intake.setExtensionGoal(IntakeState.L1_EXT)));
      }

      public static final Command autoScoreL1CoralSequence(
          Drive drive,
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake,
          Camera... cameras) {
        return Commands.sequence(
                DriveCommands.autoAlignReefCoral(drive, cameras),
                scoreL1Coral(drive, elevator, manipulator, intake))
            .finallyDo(
                () -> {
                  elevator.setPosition(() -> ReefHeight.STOW);
                  manipulator.setAlgaeArmGoal(ArmState.STOW_DOWN);
                  intake.setExtensionGoal(IntakeState.STOW);
                });
      }

      public static final Command postL1Score(
          Elevator elevator, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
        return Commands.sequence(
            elevator.setPosition(() -> ReefHeight.STOW),
            manipulator.setAlgaeArmGoal(ArmState.STOW_DOWN),
            intake.setExtensionGoal(IntakeState.STOW));
      }

      public static final Command scoreCoralSequence(
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake,
          BooleanSupplier autoAligned) {
        return Commands.sequence(
            Commands.either(
                DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    () -> ReefHeight.L3,
                    ArmState.STOW_DOWN,
                    IntakeState.STOW),
                DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    () -> RobotState.getOIData().currentReefHeight(),
                    ArmState.STOW_DOWN,
                    IntakeState.STOW),
                () ->
                    RobotState.getOIData().currentReefHeight().equals(ReefHeight.L4)
                        && !elevator.getPosition().equals(ElevatorPositions.L4)),
            Commands.waitUntil(() -> autoAligned.getAsBoolean()),
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                () -> RobotState.getOIData().currentReefHeight(),
                ArmState.STOW_DOWN,
                IntakeState.STOW),
            Commands.parallel(
                Commands.either(
                    manipulator.scoreL4Coral().withTimeout(0.4),
                    manipulator.scoreCoral().withTimeout(0.15),
                    () -> RobotState.getOIData().currentReefHeight().equals(ReefHeight.L4))),
            DecisionTree.moveSequence(
                    elevator,
                    manipulator,
                    intake,
                    () -> ReefHeight.STOW,
                    ArmState.STOW_DOWN,
                    IntakeState.STOW)
                .onlyIf(
                    () ->
                        elevator.getPosition().equals(ElevatorPositions.L3)
                            || elevator.getPosition().equals(ElevatorPositions.L2)));
      }

      public static final Command autoScoreCoralSequence(
          Drive drive,
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake,
          Camera... cameras) {

        return Commands.either(
            Commands.sequence(
                autoScoreL1CoralSequence(drive, elevator, manipulator, intake, cameras),
                postL1Score(elevator, manipulator, intake)),
            Commands.sequence(
                Commands.either(
                    DecisionTree.moveSequence(
                        elevator,
                        manipulator,
                        intake,
                        () -> ReefHeight.L2,
                        ArmState.STOW_DOWN,
                        IntakeState.STOW),
                    Commands.none(),
                    () ->
                        RobotState.getOIData().currentReefHeight().equals(ReefHeight.L1)
                            || RobotState.getOIData().currentReefHeight().equals(ReefHeight.STOW)
                            || RobotState.getOIData()
                                .currentReefHeight()
                                .equals(ReefHeight.CORAL_INTAKE)),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    scoreCoralSequence(
                        elevator,
                        manipulator,
                        intake,
                        () -> RobotState.getReefAlignData().atCoralSetpoint())),
                Commands.sequence(
                        DecisionTree.moveSequence(
                            elevator,
                            manipulator,
                            intake,
                            () -> ReefHeight.L4_PLUS,
                            ArmState.STOW_DOWN,
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
          Supplier<ReefHeight> level,
          Camera... cameras) {
        return Commands.sequence(
            Commands.parallel(
                    DriveCommands.autoAlignReefAlgae(drive, cameras),
                    Commands.sequence(
                        DecisionTree.moveSequence(
                            elevator,
                            manipulator,
                            intake,
                            level,
                            ArmState.STOW_DOWN,
                            IntakeState.STOW),
                        DecisionTree.moveSequence(
                            elevator,
                            manipulator,
                            intake,
                            level,
                            ArmState.REEF_INTAKE,
                            IntakeState.STOW)),
                    manipulator.intakeReefAlgae())
                .until(() -> RobotState.isHasAlgae()),
            Commands.parallel(
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    Commands.either(
                        DecisionTree.moveSequence(
                            elevator,
                            manipulator,
                            intake,
                            () -> ReefHeight.STOW,
                            ArmState.STOW_UP,
                            IntakeState.STOW),
                        Commands.none(),
                        RobotState::isHasAlgae)),
                Commands.runEnd(
                        () -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)),
                        () -> drive.stop())
                    .withTimeout(0.5)));
      }

      public static final Command scoreAlgae(
          Elevator elevator, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
        return Commands.sequence(
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                () -> ReefHeight.ALGAE_SCORE,
                ArmState.STOW_UP,
                IntakeState.STOW),
            Commands.waitSeconds(0.5),
            manipulator.scoreAlgae().withTimeout(0.5));
      }

      public static final Command dropAlgae(
          Drive drive,
          Elevator elevator,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake,
          Supplier<ReefHeight> level,
          Camera... cameras) {
        return Commands.sequence(
            DriveCommands.autoAlignReefAlgae(drive, cameras),
            Commands.deadline(
                Commands.sequence(
                    DecisionTree.moveSequence(
                        elevator, manipulator, intake, level, ArmState.STOW_DOWN, IntakeState.STOW),
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
            manipulator.scoreAlgae().withTimeout(0.75),
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                () -> ReefHeight.CORAL_INTAKE,
                ArmState.STOW_DOWN,
                IntakeState.STOW));
      }

      public static final Command netHeight(
          Elevator elevator,
          Funnel funnel,
          V2_RedundancyManipulator manipulator,
          V2_RedundancyIntake intake) {
        return Commands.sequence(
            funnel.setClapDaddyGoal(FunnelState.CLOSED),
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                () -> ReefHeight.ALGAE_SCORE,
                ArmState.STOW_UP,
                IntakeState.STOW));
      }

      public static final Command scoreProcessorNew(
          Elevator elevator, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
        return Commands.sequence(
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                () -> ReefHeight.ALGAE_FLOOR_INTAKE,
                ArmState.STOW_DOWN,
                IntakeState.INTAKE),
            intake.setRollerVoltage(-6));
      }

      public static final Command scoreProcessor(
          Elevator elevator, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
        return DecisionTree.moveSequence(
            elevator,
            manipulator,
            intake,
            () -> ReefHeight.STOW,
            ArmState.PROCESSOR,
            IntakeState.STOW);
      }

      public static final Command floorIntakeSequence(
          V2_RedundancyManipulator manipulator, Elevator elevator, V2_RedundancyIntake intake) {
        return Commands.sequence(
            Commands.sequence(
                    Commands.deadline(
                        DecisionTree.moveSequence(
                            elevator,
                            manipulator,
                            intake,
                            () -> ReefHeight.ALGAE_FLOOR_INTAKE,
                            ArmState.FLOOR_INTAKE,
                            IntakeState.INTAKE),
                        Commands.runOnce(() -> RobotState.setHasAlgae(false)),
                        intake.setRollerVoltage(6.0)),
                    Commands.parallel(intake.intakeAlgae(), manipulator.intakeFloorAlgae()))
                .until(() -> RobotState.isHasAlgae()));
      }

      public static final Command postFloorIntakeSequence(
          V2_RedundancyManipulator manipulator, Elevator elevator, V2_RedundancyIntake intake) {
        return Commands.either(
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                () -> ReefHeight.STOW,
                ArmState.STOW_UP,
                IntakeState.STOW),
            DecisionTree.moveSequence(
                elevator,
                manipulator,
                intake,
                () -> ReefHeight.STOW,
                ArmState.STOW_DOWN,
                IntakeState.STOW),
            RobotState::isHasAlgae);
      }
    }
  }
}
