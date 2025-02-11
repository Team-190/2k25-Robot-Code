package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevator;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnel;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulator;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {
  // Shared
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

  public static class IntakeCommands {
    public static final Command intakeCoral(
        V1_GammaElevator elevator, V1_GammaFunnel funnel, V1_GammaManipulator manipulator) {
      return Commands.sequence(
          elevator.setPosition(ReefHeight.INTAKE),
          Commands.waitUntil(elevator::atGoal),
          Commands.race(
              manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())));
    }
  }

  public static class ScoreCommands {
    public static final Command autoScoreCoral(
        Drive drive,
        V1_GammaElevator elevator,
        V1_GammaFunnel funnel,
        V1_GammaManipulator manipulator,
        ReefHeight level,
        Camera... cameras) {
      return Commands.sequence(
          DriveCommands.alignRobotToAprilTag(drive, cameras),
          // elevator.setPosition(level),
          // Commands.waitUntil(() -> elevator.atGoal()),
          manipulator.scoreCoral().withTimeout(0.5));
    }

    public static final Command scoreCoral(
        V1_GammaElevator elevator, V1_GammaManipulator manipulator, ReefHeight level) {
      return Commands.sequence(
          elevator.setPosition(level),
          Commands.waitUntil(() -> elevator.atGoal()),
          manipulator.scoreCoral().withTimeout(0.5));
    }
  }
}
