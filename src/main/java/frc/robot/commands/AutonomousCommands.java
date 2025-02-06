package frc.robot.commands;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevator;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnel;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulator;
import java.util.ArrayList;
import java.util.Arrays;

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
                B_LEFT_PATH3.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return autoBLeft;
  }

  public static final AutoRoutine autoBRight(Drive drive) {
    AutoRoutine autoBRight = drive.getAutoFactory().newRoutine("autoBLeft");

    AutoTrajectory B_RIGHT_PATH1 = mirrorAuto("B_LEFT_PATH1", autoBRight);
    AutoTrajectory B_RIGHT_PATH2 = mirrorAuto("B_LEFT_PATH2", autoBRight);
    AutoTrajectory B_RIGHT_PATH3 = mirrorAuto("B_LEFT_PATH3", autoBRight);

    autoBRight
        .active()
        .onTrue(
            Commands.sequence(
                B_RIGHT_PATH1.resetOdometry(),
                B_RIGHT_PATH1.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                DriveCommands.alignRobotToAprilTag(drive),
                B_RIGHT_PATH2.cmd(),
                B_RIGHT_PATH3.cmd(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                DriveCommands.alignRobotToAprilTag(drive)));

    return autoBRight;
  }

  public static final AutoTrajectory mirrorAuto(String traj, AutoRoutine routine) {
    Trajectory<SwerveSample> trajectory = routine.trajectory(traj).getRawTrajectory();
    var flippedStates = new ArrayList<SwerveSample>();
    for (var sample : trajectory.samples()) {
      flippedStates.add(flipSample(sample));
    }
    AutoTrajectory mirrored =
        routine.trajectory(
            new Trajectory<SwerveSample>(
                trajectory.name(), flippedStates, trajectory.splits(), trajectory.events()));
    return mirrored;
  }

  private static final SwerveSample flipSample(SwerveSample sample) {
    return new SwerveSample(
      sample.t,
      sample.y,  // Swap x with y
      -sample.x, // Swap y with x and negate to reflect across the y-axis
      sample.heading + Math.PI / 2, // Rotate heading by +90 degrees
      sample.vy,  // Swap vx with vy
      -sample.vx, // Swap vy with vx and negate
      sample.omega, // Omega stays the same (no mirror effect needed)
      sample.ay,  // Swap ax with ay
      -sample.ax, // Swap ay with ax and negate
      sample.alpha, // Alpha stays the same
      sample.moduleForcesY(), // Swap forces in x with y
      Arrays.stream(sample.moduleForcesX()).map(x -> -x).toArray() // Swap and negate forces in y
  );  
  }
}
