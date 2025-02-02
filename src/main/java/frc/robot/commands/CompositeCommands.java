package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevator;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorConstants;
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
          elevator.setPosition(V1_GammaElevatorConstants.ElevatorPositions.INTAKE),
          Commands.waitUntil(() -> elevator.atGoal()),
          Commands.race(
              manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())));
    }
  }

  public static class ScoreCommands {}
}
