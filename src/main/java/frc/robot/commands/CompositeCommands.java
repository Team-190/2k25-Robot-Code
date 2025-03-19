package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.shared.funnel.Funnel;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.leds.V1_StackUp_LEDs;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.BooleanSupplier;

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
              Commands.runOnce(() -> V1_StackUp_LEDs.setIntaking(true)),
              elevator.setPosition(ReefHeight.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.race(
                  manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())))
          .finallyDo(() -> V1_StackUp_LEDs.setIntaking(false));
    }

    public static final Command intakeCoralOverride(
        Elevator elevator, Funnel funnel, V1_StackUpManipulator manipulator) {
      return Commands.sequence(
              Commands.runOnce(() -> V1_StackUp_LEDs.setIntaking(true)),
              elevator.setPosition(ReefHeight.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
          .finallyDo(() -> V1_StackUp_LEDs.setIntaking(false));
    }

    public static final Command intakeCoral(
        Elevator elevator,
        Funnel funnel,
        V2_RedundancyManipulator manipulator,
        V2_RedundancyIntake intake) {
      return Commands.sequence(
              Commands.runOnce(() -> V1_StackUp_LEDs.setIntaking(true)),
              elevator.setPosition(ReefHeight.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.race(manipulator.intakeCoral(), funnel.intakeCoral(() -> intake.hasCoral())))
          .finallyDo(() -> V1_StackUp_LEDs.setIntaking(false));
    }

    public static final Command intakeCoralOverride(
        Elevator elevator, Funnel funnel, V2_RedundancyManipulator manipulator) {
      return Commands.sequence(
              Commands.runOnce(() -> V1_StackUp_LEDs.setIntaking(true)),
              elevator.setPosition(ReefHeight.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
          .finallyDo(() -> V1_StackUp_LEDs.setIntaking(false));
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
              elevator.setPosition(ReefHeight.L2),
              DriveCommands.autoAlignReefCoral(drive, cameras),
              scoreCoralSequence(elevator, manipulator)),
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
                    case 10, 6, 8, 21, 17, 19 -> ReefHeight.BOT_ALGAE;
                    case 9, 11, 7, 22, 20, 18 -> ReefHeight.TOP_ALGAE;
                    default -> ReefHeight.BOT_ALGAE;
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
          manipulator.setAlgaeArmGoal(armState),
          Commands.sequence(
              elevator.setPosition(ReefHeight.ALGAE_MID), manipulator.setAlgaeArmGoal(armState)),
          () ->
              elevator.getPosition().getPosition()
                  >= ElevatorConstants.ElevatorPositions.ALGAE_MID.getPosition());
    }

    public static final Command floorIntakeSequence(
        V2_RedundancyManipulator manipulator, Elevator elevator, V2_RedundancyIntake intake) {
      return Commands.sequence(
          Commands.parallel(
                  elevator.setPosition(ReefHeight.ALGAE_MID),
                  intake.intakeAlgae(),
                  manipulator.intakeAlgae())
              .until(() -> RobotState.isHasAlgae()),
          moveAlgaeArm(manipulator, elevator, ArmState.UP),
          elevator.setPosition(ReefHeight.STOW));
    }

    public static final Command scoreNetSequence(
        Elevator elevator, V1_StackUpManipulator manipulator, BooleanSupplier waitForSpit) {
      return Commands.none();
    }

    public static final Command intakeFromReefSequence( // With onTrue, and pass in button
        V2_RedundancyManipulator manipulator,
        Elevator elevator,
        Drive drive,
        BooleanSupplier waitForButton,
        Camera... cameras) {
      return Commands.none();
    }

    public static final Command dropFromReefSequence(
        V2_RedundancyManipulator manipulator,
        Elevator elevator,
        Drive drive,
        BooleanSupplier waitForButton,
        Camera... cameras) {
      return Commands.none();
    }

    public static final Command stowAll(V2_RedundancyManipulator manipulator, Elevator elevator) {
      return Commands.sequence(
          moveAlgaeArm(manipulator, elevator, ArmState.DOWN),
          elevator.setPosition(ReefHeight.STOW));
    }
  }
}
