package frc.robot.commands;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;

public class AutonomousCommands {
  public static final AutoRoutine twoPieceBack(Drive drive) {
    AutoRoutine twoPieceBack = drive.getAutoFactory().newRoutine("twoPieceBack");

    AutoTrajectory traj = twoPieceBack.trajectory("Path1");
    AutoTrajectory traj2 = twoPieceBack.trajectory("Path2");
    AutoTrajectory traj3 = twoPieceBack.trajectory("Path3");

    twoPieceBack
        .active()
        .onTrue(
            Commands.sequence(
                traj.resetOdometry(),
                Commands.runOnce(() -> RobotState.setCurrentReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                traj2.cmd(),
                traj3.cmd(),
                Commands.runOnce(() -> RobotState.setCurrentReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return twoPieceBack;
  }

  public static final AutoRoutine threePieceAntiProcessor(Drive drive) {
    AutoRoutine threePieceAntiProcessor = drive.getAutoFactory().newRoutine("twoPieceBack");

    AutoTrajectory traj = threePieceAntiProcessor.trajectory("Path1");
    AutoTrajectory traj2 = threePieceAntiProcessor.trajectory("Path2");
    AutoTrajectory traj4 = threePieceAntiProcessor.trajectory("Path4");

    threePieceAntiProcessor
        .active()
        .onTrue(
            Commands.sequence(
                traj.resetOdometry(),
                Commands.runOnce(() -> RobotState.setCurrentReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                traj2.cmd(),
                DriveCommands.alignRobotToAprilTag(drive),
                traj4.cmd(),
                Commands.runOnce(() -> RobotState.setCurrentReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return threePieceAntiProcessor;
  }
}
