package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_gamma.climber.V1_GammaClimber;
import frc.robot.subsystems.v1_gamma.climber.V1_GammaClimberConstants;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevator;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnel;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelConstants.FunnelState;
import frc.robot.subsystems.v1_gamma.leds.V1_Gamma_LEDs;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulator;
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
      V1_GammaElevator elevator, V1_GammaFunnel funnel, V1_GammaClimber climber, Drive drive) {
    return Commands.sequence(
        elevator.setPosition(ReefHeight.STOW),
        Commands.waitSeconds(0.02),
        Commands.waitUntil(elevator::atGoal),
        funnel.setClapDaddyGoal(FunnelState.CLIMB),
        Commands.parallel(
            climber.releaseClimber(),
            Commands.waitSeconds(V1_GammaClimberConstants.WAIT_AFTER_RELEASE_SECONDS)),
        Commands.waitUntil(climber::climberReady),
        Commands.deadline(climber.winchClimber(), Commands.run(drive::stop)));
  }

  public static final Command setStaticReefHeight(ReefHeight height) {
    return Commands.runOnce(() -> RobotState.setReefHeight(height));
  }

  public static final Command setDynamicReefHeight(ReefHeight height, V1_GammaElevator elevator) {
    return Commands.sequence(
        Commands.runOnce(() -> RobotState.setReefHeight(height)), elevator.setPosition());
  }

  public static class IntakeCommands {
    public static final Command intakeCoral(
        V1_GammaElevator elevator, V1_GammaFunnel funnel, V1_GammaManipulator manipulator) {
      return Commands.sequence(
              Commands.runOnce(() -> V1_Gamma_LEDs.setIntaking(true)),
              elevator.setPosition(ReefHeight.INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.race(
                  manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())))
          .finallyDo(() -> V1_Gamma_LEDs.setIntaking(false));
    }

    public static final Command twerk(
        Drive drive, V1_GammaElevator elevator, V1_GammaManipulator manipulator) {
      return Commands.sequence(
          Commands.parallel(
              DriveCommands.inchMovement(drive, -1.4, 0.1), elevator.setPosition(ReefHeight.L4)),
          Commands.waitUntil(elevator::atGoal),
          manipulator.toggleAlgaeArm(),
          Commands.waitSeconds(0.1),
          Commands.deferredProxy(
              () ->
                  elevator.setPosition(
                      switch (RobotState.getReefAlignData().closestReefTag()) {
                        case 10, 6, 8, 21, 17, 19 -> ReefHeight.BOT_ALGAE;
                        case 9, 11, 7, 22, 20, 18 -> ReefHeight.TOP_ALGAE;
                        default -> ReefHeight.BOT_ALGAE;
                      })),
          manipulator.removeAlgae().until(elevator::atGoal),
          manipulator.removeAlgae().withTimeout(0.35),
          manipulator.toggleAlgaeArm());
    }

    public static final Command twerk(
        Drive drive, V1_GammaElevator elevator, V1_GammaManipulator manipulator, ReefHeight level) {
      return Commands.sequence(
          Commands.parallel(
              DriveCommands.inchMovement(drive, -1.4, 0.1), elevator.setPosition(ReefHeight.L4)),
          Commands.waitUntil(elevator::atGoal),
          manipulator.toggleAlgaeArm(),
          Commands.waitSeconds(0.1),
          Commands.deferredProxy(() -> elevator.setPosition(level)),
          manipulator.removeAlgae().until(elevator::atGoal),
          manipulator.removeAlgae().withTimeout(0.35),
          manipulator.toggleAlgaeArm());
    }

    public static final Command intakeCoralOverride(
        V1_GammaElevator elevator, V1_GammaFunnel funnel, V1_GammaManipulator manipulator) {
      return Commands.sequence(
              Commands.runOnce(() -> V1_Gamma_LEDs.setIntaking(true)),
              elevator.setPosition(ReefHeight.INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
          .finallyDo(() -> V1_Gamma_LEDs.setIntaking(false));
    }
  }

  public static class ScoreCommands {
    public static final Command emergencyEject(
        V1_GammaElevator elevator, V1_GammaManipulator manipulator) {
      return Commands.sequence(
          elevator.setPosition(ReefHeight.L1),
          Commands.waitSeconds(0.125),
          Commands.waitUntil(elevator::atGoal),
          manipulator.scoreCoral().withTimeout(0.4),
          elevator.setPosition(ReefHeight.STOW));
    }

    public static final Command scoreCoral(
        V1_GammaElevator elevator, V1_GammaManipulator manipulator) {
      return manipulator.scoreCoral().withTimeout(0.4);
    }

    public static final Command scoreCoralSequence(
        V1_GammaElevator elevator, V1_GammaManipulator manipulator) {
      return Commands.sequence(
          elevator.setPosition(),
          Commands.waitSeconds(0.125),
          Commands.waitUntil(elevator::atGoal),
          manipulator.scoreCoral().withTimeout(0.4));
    }

    public static final Command autoScoreCoralSequence(
        Drive drive,
        V1_GammaElevator elevator,
        V1_GammaFunnel funnel,
        V1_GammaManipulator manipulator,
        ReefHeight level,
        Camera... cameras) {
      return Commands.sequence(
          elevator.setPosition(ReefHeight.L2),
          DriveCommands.alignRobotToAprilTag(drive, cameras),
          scoreCoralSequence(elevator, manipulator));
    }
  }
}
