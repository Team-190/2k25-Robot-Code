package frc.robot.commands;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands.IntakeCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.vision.Camera;
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
      V1_GammaManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoALeft = drive.getAutoFactory().newRoutine("autoALeft");

    AutoTrajectory A_LEFT_PATH1 = autoALeft.trajectory("A_LEFT_PATH1");
    AutoTrajectory A_LEFT_PATH2 = autoALeft.trajectory("A_LEFT_PATH2");
    AutoTrajectory A_LEFT_PATH3 = autoALeft.trajectory("A_LEFT_PATH3");
    AutoTrajectory A_LEFT_PATH4 = autoALeft.trajectory("A_LEFT_PATH4");

    autoALeft
        .active()
        .onTrue(
            Commands.sequence(
                A_LEFT_PATH1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)),
                A_LEFT_PATH1.cmd(),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_LEFT_PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT))),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_LEFT_PATH3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT))),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_LEFT_PATH4.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT))),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW)));

    return autoALeft;
  }

  public static final AutoRoutine autoARight(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoARight = drive.getAutoFactory().newRoutine("autoARight");

    AutoTrajectory A_RIGHT_PATH1 = mirrorAuto("A_LEFT_PATH1", autoARight);
    AutoTrajectory A_RIGHT_PATH2 = mirrorAuto("A_LEFT_PATH2", autoARight);
    AutoTrajectory A_RIGHT_PATH3 = mirrorAuto("A_LEFT_PATH3", autoARight);
    AutoTrajectory A_RIGHT_PATH4 = mirrorAuto("A_LEFT_PATH4", autoARight);

    autoARight
        .active()
        .onTrue(
            Commands.sequence(
                A_RIGHT_PATH1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)),
                A_RIGHT_PATH1.cmd(),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_RIGHT_PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT))),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_RIGHT_PATH3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT))),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_RIGHT_PATH4.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT))),
                Commands.parallel(
                    DriveCommands.alignRobotToAprilTag(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW)));

    return autoARight;
  }

  private static final AutoTrajectory mirrorAuto(String traj, AutoRoutine routine) {
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

    return new SwerveSample( // Change y
        sample.t,
        sample.x,
        FieldConstants.fieldWidth - sample.y,
        -sample.heading,
        sample.vx,
        -sample.vy,
        -sample.omega,
        sample.ax,
        -sample.ay,
        sample.alpha,
        sample.moduleForcesX(),
        Arrays.stream(sample.moduleForcesY()).map(y -> -y).toArray());
  }
}
