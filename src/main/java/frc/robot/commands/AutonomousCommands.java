package frc.robot.commands;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;

public class AutonomousCommands {
  public static final AutoRoutine test(Drive drive) {
    AutoRoutine test = drive.getAutoFactory().newRoutine("test");

    AutoTrajectory traj = test.trajectory("Path1");
    AutoTrajectory traj2 = test.trajectory("Path2");
    AutoTrajectory traj3 = test.trajectory("Path3");

    test.active()
        .onTrue(
            Commands.sequence(
                traj.resetOdometry(),
                // traj.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                traj2.cmd(),
                traj3.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return test;
  }
}
