package frc.robot.commands;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevator;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnel;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulator;

public class AutonomousCommands {
  public static final AutoRoutine autoALeft(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator) {
    AutoRoutine autoALeft = drive.getAutoFactory().newRoutine("autoALeft");

    AutoTrajectory A_LEFT_PATH1 = autoALeft.trajectory("A_LEFT_PATH1");
    AutoTrajectory A_LEFT_PATH2 = autoALeft.trajectory("A_LEFT_PATH2");
    AutoTrajectory A_LEFT_PATH3 = autoALeft.trajectory("A_LEFT_PATH3");

    autoALeft
        .active()
        .onTrue(
            Commands.sequence(
                A_LEFT_PATH1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive), elevator.setPosition(ReefHeight.L4)),
                elevator.setPosition(ReefHeight.STOW),
                manipulator.scoreCoral().withTimeout(0.5),
                A_LEFT_PATH2.cmd(),
                Commands.runOnce(() -> drive.stop()),
                funnel.intakeCoral(() -> false).withTimeout(1),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive), elevator.setPosition(ReefHeight.L4)),
                elevator.setPosition(ReefHeight.STOW),
                A_LEFT_PATH3.cmd(),
                Commands.runOnce(() -> drive.stop()),
                funnel.intakeCoral(() -> false).withTimeout(1),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive), elevator.setPosition(ReefHeight.L4)),
                elevator.setPosition(ReefHeight.STOW)));

    return autoALeft;
  }

  public static final AutoRoutine autoARight(Drive drive) {
    AutoRoutine autoARight = drive.getAutoFactory().newRoutine("autoARight");

    AutoTrajectory A_RIGHT_PATH1 = autoARight.trajectory("A_RIGHT_PATH1");
    AutoTrajectory A_RIGHT_PATH2 = autoARight.trajectory("A_RIGHT_PATH2");
    AutoTrajectory A_RIGHT_PATH3 = autoARight.trajectory("A_RIGHT_PATH3");

    autoARight
        .active()
        .onTrue(
            Commands.sequence(
                A_RIGHT_PATH1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                A_RIGHT_PATH2.cmd(),
                DriveCommands.alignRobotToAprilTag(drive),
                A_RIGHT_PATH3.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return autoARight;
  }
}
