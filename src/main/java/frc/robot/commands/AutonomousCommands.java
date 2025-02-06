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
  public static final AutoRoutine twoPieceBack(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator) {
    AutoRoutine twoPieceBack = drive.getAutoFactory().newRoutine("twoPieceBack");

    AutoTrajectory traj = twoPieceBack.trajectory("Path1");
    AutoTrajectory traj2 = twoPieceBack.trajectory("Path2");
    AutoTrajectory traj3 = twoPieceBack.trajectory("Path3");

    twoPieceBack
        .active()
        .onTrue(
            Commands.sequence(
                traj.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                traj2.cmd(),
                Commands.runOnce(() -> drive.stop()),
                funnel.intakeCoral(() -> false).withTimeout(1.0),
                traj3.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive), elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(.25),
                elevator.setPosition(ReefHeight.STOW)));

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
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                traj2.cmd(),
                DriveCommands.alignRobotToAprilTag(drive),
                traj4.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return threePieceAntiProcessor;
  }

  public static final AutoRoutine threePieceTakeTwoLeft(Drive drive) {
    AutoRoutine threePieceTakeTwoLeft = drive.getAutoFactory().newRoutine("threePieceTakeTwoLeft");

    AutoTrajectory traj5 = threePieceTakeTwoLeft.trajectory("Path5");
    AutoTrajectory traj6 = threePieceTakeTwoLeft.trajectory("Path6");
    AutoTrajectory traj7 = threePieceTakeTwoLeft.trajectory("Path7");

    threePieceTakeTwoLeft
        .active()
        .onTrue(
            Commands.sequence(
                traj5.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                traj6.cmd(),
                DriveCommands.alignRobotToAprilTag(drive),
                traj7.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return threePieceTakeTwoLeft;
  }

  public static final AutoRoutine threePieceTakeTwoRight(Drive drive) {
    AutoRoutine threePieceTakeTwoRight =
        drive.getAutoFactory().newRoutine("threePieceTakeTwoRight");

    AutoTrajectory traj8 = threePieceTakeTwoRight.trajectory("Path8");
    AutoTrajectory traj9 = threePieceTakeTwoRight.trajectory("Path9");
    AutoTrajectory traj10 = threePieceTakeTwoRight.trajectory("Path10");

    threePieceTakeTwoRight
        .active()
        .onTrue(
            Commands.sequence(
                traj8.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                traj9.cmd(),
                DriveCommands.alignRobotToAprilTag(drive),
                traj10.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return threePieceTakeTwoRight;
  }

  public static final AutoRoutine autoALeft(Drive drive) {
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
                DriveCommands.alignRobotToAprilTag(drive),
                A_LEFT_PATH2.cmd(),
                DriveCommands.alignRobotToAprilTag(drive),
                A_LEFT_PATH3.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

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

  public static final AutoRoutine autoBLeft(Drive drive) {
    AutoRoutine autoBLeft = drive.getAutoFactory().newRoutine("autoBLeft");

    AutoTrajectory B_LEFT_PATH1 = autoBLeft.trajectory("B_LEFT_PATH1");
    AutoTrajectory B_LEFT_PATH2 = autoBLeft.trajectory("B_LEFT_PATH2");
    AutoTrajectory B_LEFT_PATH3 = autoBLeft.trajectory("B_LEFT_PATH3");

    autoBLeft
        .active()
        .onTrue(
            Commands.sequence(
                B_LEFT_PATH1.resetOdometry(),
                B_LEFT_PATH1.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                B_LEFT_PATH2.cmd(),
                Commands.waitSeconds(2),
                B_LEFT_PATH3.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return autoBLeft;
  }
}
