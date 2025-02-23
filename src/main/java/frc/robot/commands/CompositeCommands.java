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
        Commands.waitUntil(elevator::atGoal),
        funnel.setClapDaddyGoal(FunnelState.CLIMB),
        climber.releaseClimber(),
        Commands.waitSeconds(V1_GammaClimberConstants.WAIT_AFTER_RELEASE_SECONDS),
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
          DriveCommands.alignRobotToAprilTag(drive, cameras),
          scoreCoralSequence(elevator, manipulator));
    }
  }
}
