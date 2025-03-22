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
import frc.robot.subsystems.shared.elevator.ElevatorConstants;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.funnel.Funnel;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {
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

  public static class IntakeCommands {
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

    public static final Command intakeCoral(
        Elevator elevator,
        Funnel funnel,
        V2_RedundancyManipulator manipulator,
        V2_RedundancyIntake intake) {
      return Commands.sequence(
              Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
              Commands.either(
                  Commands.none(),
                  AlgaeCommands.stowAll(manipulator, elevator),
                  () -> manipulator.getState().equals(ArmState.DOWN)),
              elevator.setPosition(ReefHeight.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.race(manipulator.intakeCoral(), funnel.intakeCoral(() -> intake.hasCoral()))
                  .until(() -> intake.hasCoral()))
          .finallyDo(() -> RobotState.setIntakingCoral(false));
    }

    public static final Command intakeCoralOverride(
        Elevator elevator, Funnel funnel, V2_RedundancyManipulator manipulator) {
      return Commands.sequence(
              Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
              Commands.either(
                  Commands.none(),
                  AlgaeCommands.stowAll(manipulator, elevator),
                  () -> manipulator.getState().equals(ArmState.DOWN)),
              elevator.setPosition(ReefHeight.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
          .finallyDo(() -> RobotState.setIntakingCoral(false));
    }

    public static final Command intakeCoralCloseOverride(
        Elevator elevator, Funnel funnel, V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake) {
      return Commands.sequence(
              Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
              Commands.either(
                  Commands.none(),
                  AlgaeCommands.stowAll(manipulator, elevator),
                  () -> manipulator.getState().equals(ArmState.DOWN)),
              elevator.setPosition(ReefHeight.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.parallel(manipulator.intakeCoral(), funnel.funnelClosedOverride()).until(intake::hasCoral))
          .finallyDo(() -> RobotState.setIntakingCoral(false));
    }
  }

  public static class ScoreCommands {

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

    public static final Command emergencyEject(
        Elevator elevator, V2_RedundancyManipulator manipulator) {
      return Commands.sequence(
          elevator.setPosition(ReefHeight.L1),
          Commands.waitSeconds(0.125),
          Commands.waitUntil(elevator::atGoal),
          manipulator.scoreCoral().withTimeout(0.4),
          elevator.setPosition(ReefHeight.STOW));
    }

    public static final Command scoreCoral(V2_RedundancyManipulator manipulator) {
      return manipulator.scoreCoral().withTimeout(0.4);
    }

    public static final Command scoreCoralSequence(
        Elevator elevator, V2_RedundancyManipulator manipulator) {
      return Commands.sequence(
          elevator.setPosition(),
          Commands.waitSeconds(0.125),
          Commands.waitUntil(elevator::atGoal),
          manipulator.scoreCoral().withTimeout(0.4));
    }

    public static final Command autoScoreCoralSequence(
        Drive drive, Elevator elevator, V2_RedundancyManipulator manipulator, Camera... cameras) {
      return Commands.either(
          autoScoreL1CoralSequence(drive, elevator, manipulator, cameras),
          Commands.sequence(
              elevator
                  .setPosition(ReefHeight.L2)
                  .onlyIf(() -> elevator.getPosition().equals(ElevatorPositions.STOW)),
              DriveCommands.autoAlignReefCoral(drive, cameras),
              scoreCoralSequence(elevator, manipulator),
              Commands.waitSeconds(0.25),
              Commands.sequence(
                      elevator.setPosition(ReefHeight.L4_PLUS),
                      manipulator.scoreCoral().withTimeout(0.5))
                  .onlyIf(() -> elevator.getPosition().equals(ElevatorPositions.L4))),
          () -> RobotState.getOIData().currentReefHeight().equals(ReefHeight.L1));
    }

    public static final Command autoScoreL1CoralSequence(
        Drive drive, Elevator elevator, V2_RedundancyManipulator manipulator, Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefCoral(drive, cameras),
          scoreL1Coral(drive, elevator, manipulator));
    }

    public static final Command scoreL1Coral(
        Drive drive, Elevator elevator, V2_RedundancyManipulator manipulator) {
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
  }

  public static class AlgaeCommands {
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

    public static final Command moveAlgaeArm(
        V2_RedundancyManipulator manipulator, Elevator elevator, ArmState armState) {
      return Commands.either(
          Commands.sequence(
              manipulator.setAlgaeArmGoal(armState), manipulator.waitUntilAlgaeArmAtGoal()),
          Commands.sequence(
              Commands.sequence(
                  elevator.setPosition(ReefHeight.ALGAE_MID), elevator.waitUntilAtGoal()),
              manipulator.setAlgaeArmGoal(armState)),
          () ->
              elevator.getPosition().getPosition()
                  >= ElevatorConstants.ElevatorPositions.ALGAE_MID.getPosition());
    }

    public static final Command dropFromReefSequence(
        V2_RedundancyManipulator manipulator,
        Elevator elevator,
        Drive drive,
        ReefHeight level,
        Camera... cameras) {
      return Commands.sequence(
              DriveCommands.autoAlignReefAlgae(drive, cameras),
              Commands.deadline(
                  Commands.sequence(
                      elevator.setPosition(level),
                      elevator.waitUntilAtGoal(),
                      manipulator.setAlgaeArmGoal(ArmState.REEF_INTAKE),
                      manipulator.waitUntilAlgaeArmAtGoal(),
                      Commands.waitSeconds(1.0),
                      Commands.runEnd(
                              () -> drive.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0)),
                              () -> drive.stop())
                          .withTimeout(0.5)),
                  manipulator.runManipulator(6)),
              manipulator.runManipulator(-3).withTimeout(0.5))
          .finallyDo(() -> stowAll(manipulator, elevator));
    }

    public static final Command dropFromReefSequence(
        V2_RedundancyManipulator manipulator, Elevator elevator, Drive drive, Camera... cameras) {
      return Commands.deferredProxy(
          () ->
              dropFromReefSequence(
                  manipulator,
                  elevator,
                  drive,
                  switch (RobotState.getReefAlignData().closestReefTag()) {
                    case 10, 6, 8, 21, 17, 19 -> ReefHeight.ALGAE_INTAKE_BOTTOM;
                    case 9, 11, 7, 22, 20, 18 -> ReefHeight.ALGAE_INTAKE_TOP;
                    default -> ReefHeight.ALGAE_INTAKE_BOTTOM;
                  },
                  cameras));
    }

    public static final Command intakeFromReefSequence(
        V2_RedundancyManipulator manipulator,
        Elevator elevator,
        Drive drive,
        ReefHeight level,
        Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefAlgae(drive, cameras),
          Commands.deadline(
              Commands.sequence(
                  elevator.setPosition(level),
                  elevator.waitUntilAtGoal(),
                  manipulator.setAlgaeArmGoal(ArmState.REEF_INTAKE),
                  manipulator.waitUntilAlgaeArmAtGoal(),
                  Commands.waitSeconds(1.0),
                  Commands.runEnd(
                          () -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)),
                          () -> drive.stop())
                      .withTimeout(0.5)),
              manipulator.intakeAlgae()),
          Commands.either(
              Commands.none(),
              Commands.sequence(
                  moveAlgaeArm(manipulator, elevator, ArmState.DOWN),
                  elevator.setPosition(ReefHeight.STOW)),
              RobotState::isHasAlgae));
    }

    public static final Command intakeFromReefSequence(
        V2_RedundancyManipulator manipulator, Elevator elevator, Drive drive, Camera... cameras) {
      return Commands.deferredProxy(
          () ->
              intakeFromReefSequence(
                  manipulator,
                  elevator,
                  drive,
                  switch (RobotState.getReefAlignData().closestReefTag()) {
                    case 10, 6, 8, 21, 17, 19 -> ReefHeight.ALGAE_INTAKE_BOTTOM;
                    case 9, 11, 7, 22, 20, 18 -> ReefHeight.ALGAE_INTAKE_TOP;
                    default -> ReefHeight.ALGAE_INTAKE_BOTTOM;
                  },
                  cameras));
    }

    public static final Command stowAll(V2_RedundancyManipulator manipulator, Elevator elevator) {
      return Commands.sequence(
          moveAlgaeArm(manipulator, elevator, ArmState.DOWN),
          manipulator.waitUntilAlgaeArmAtGoal(),
          elevator.setPosition(ReefHeight.STOW));
    }
  }

  public static Command testAlgae(Elevator elevator, V2_RedundancyManipulator manipulator) {
    return Commands.sequence(
        elevator.setPosition(ReefHeight.ALGAE_MID),
        elevator.waitUntilAtGoal(),
        manipulator.setAlgaeArmGoal(ArmState.REEF_INTAKE));
  }

  public static final Command scoreAlgae(V2_RedundancyManipulator manipulator) {
    return Commands.sequence(manipulator.scoreAlgae());
  }
}
