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
  public static final AutoRoutine test(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator) {
    AutoRoutine test = drive.getAutoFactory().newRoutine("test");

    AutoTrajectory traj = test.trajectory("Path1");
    AutoTrajectory traj2 = test.trajectory("Path2");
    AutoTrajectory traj3 = test.trajectory("Path3");

    test.active()
        .onTrue(
            Commands.sequence(
                traj.resetOdometry(),
                traj.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive), elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(.25),
                elevator.setPosition(ReefHeight.STOW),
                traj2.cmd(),
                Commands.runOnce(() -> drive.stop()),
                funnel.intakeCoral(() -> false).withTimeout(1.0),
                traj3.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive), elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(.25),
                elevator.setPosition(ReefHeight.STOW)));

    return test;
  }
}
