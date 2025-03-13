package frc.robot.commands;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands.IntakeCommands;
import frc.robot.commands.CompositeCommands.ScoreCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevator;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnel;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulator;

public class AutonomousCommands {
  public static final AutoRoutine Left3(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator,
      Camera... cameras) {
    AutoRoutine left3 = drive.getAutoFactory().newRoutine("Left3");

    AutoTrajectory PATH_1 = left3.trajectory("3Left1");
    AutoTrajectory PATH_2 = left3.trajectory("3Left2");
    AutoTrajectory PATH_3 = left3.trajectory("3Left3");

    left3
        .active()
        .onTrue(
            Commands.sequence(
                PATH_1.resetOdometry(),
                Commands.runOnce(
                    () -> {
                      RobotState.setReefPost(ReefPose.RIGHT);
                      RobotState.setReefHeight(ReefHeight.L4);
                    }),
                PATH_1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                ScoreCommands.descoreAlgae(drive, elevator, manipulator, cameras)));

    return left3;
  }

  public static final AutoRoutine Right3(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator,
      Camera... cameras) {
    AutoRoutine right3 = drive.getAutoFactory().newRoutine("Right3");

    AutoTrajectory PATH_1 = right3.trajectory("3Right1");
    AutoTrajectory PATH_2 = right3.trajectory("3Right2");
    AutoTrajectory PATH_3 = right3.trajectory("3Right3");

    right3
        .active()
        .onTrue(
            Commands.sequence(
                PATH_1.resetOdometry(),
                Commands.runOnce(
                    () -> {
                      RobotState.setReefPost(ReefPose.LEFT);
                      RobotState.setReefHeight(ReefHeight.L4);
                    }),
                PATH_1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                ScoreCommands.descoreAlgae(drive, elevator, manipulator, ReefHeight.L4, cameras)));

    return right3;
  }

  public static final AutoRoutine Left2(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator,
      Camera... cameras) {
    AutoRoutine left2 = drive.getAutoFactory().newRoutine("Left2");

    AutoTrajectory PATH_1 = left2.trajectory("2Left1");
    AutoTrajectory PATH_2 = left2.trajectory("2Left2");

    left2
        .active()
        .onTrue(
            Commands.sequence(
                PATH_1.resetOdometry(),
                Commands.runOnce(
                    () -> {
                      RobotState.setReefPost(ReefPose.LEFT);
                      RobotState.setReefHeight(ReefHeight.L4);
                    }),
                PATH_1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW)));

    return left2;
  }

  public static final AutoRoutine Right2(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator,
      Camera... cameras) {
    AutoRoutine right2 = drive.getAutoFactory().newRoutine("Right2");

    AutoTrajectory PATH_1 = right2.trajectory("2Right1");
    AutoTrajectory PATH_2 = right2.trajectory("2Right2");

    right2
        .active()
        .onTrue(
            Commands.sequence(
                PATH_1.resetOdometry(),
                Commands.runOnce(
                    () -> {
                      RobotState.setReefPost(ReefPose.RIGHT);
                      RobotState.setReefHeight(ReefHeight.L4);
                    }),
                PATH_1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW)));

    return right2;
  }

  public static final AutoRoutine Left4(
      Drive drive,
      V1_GammaElevator elevator,
      V1_GammaFunnel funnel,
      V1_GammaManipulator manipulator,
      Camera... cameras) {
    AutoRoutine left4 = drive.getAutoFactory().newRoutine("Left4");

    AutoTrajectory PATH_1 = left4.trajectory("4Left1");
    AutoTrajectory PATH_2 = left4.trajectory("4Left2");
    AutoTrajectory PATH_3 = left4.trajectory("4Left3");
    AutoTrajectory PATH_4 = left4.trajectory("4Left4");

    left4
        .active()
        .onTrue(
            Commands.sequence(
                PATH_1.resetOdometry(),
                Commands.runOnce(
                    () -> {
                      RobotState.setReefPost(ReefPose.RIGHT);
                      RobotState.setReefHeight(ReefHeight.L4);
                    }),
                PATH_1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH_4.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));

    return left4;
  }
}
