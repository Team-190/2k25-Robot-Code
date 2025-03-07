package frc.robot.commands;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands.IntakeCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.elevator.V1_StackUpElevator;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnel;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;

public class AutonomousCommands {
  public static final AutoRoutine autoALeft(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoALeft = drive.getAutoFactory().newRoutine("autoALeft");

    AutoTrajectory A_LEFT_PATH1 = autoALeft.trajectory("A_LEFT_PATH1");
    AutoTrajectory A_LEFT_PATH2 = autoALeft.trajectory("A_LEFT_PATH2");
    AutoTrajectory A_LEFT_PATH3 = autoALeft.trajectory("A_LEFT_PATH3");

    autoALeft
        .active()
        .onTrue(
            Commands.sequence(
                A_LEFT_PATH1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                A_LEFT_PATH1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_LEFT_PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_LEFT_PATH3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));

    return autoALeft;
  }

  public static final AutoRoutine autoARight(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoARight = drive.getAutoFactory().newRoutine("autoARight");

    AutoTrajectory A_RIGHT_PATH1 = autoARight.trajectory("A_RIGHT_PATH1");
    AutoTrajectory A_RIGHT_PATH2 = autoARight.trajectory("A_RIGHT_PATH2");
    AutoTrajectory A_RIGHT_PATH3 = autoARight.trajectory("A_RIGHT_PATH3");

    autoARight
        .active()
        .onTrue(
            Commands.sequence(
                A_RIGHT_PATH1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                A_RIGHT_PATH1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_RIGHT_PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    A_RIGHT_PATH3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));

    return autoARight;
  }

  public static final AutoRoutine autoBLeft(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoBLeft = drive.getAutoFactory().newRoutine("autoBLeft");

    AutoTrajectory B_LEFT_PATH1 = autoBLeft.trajectory("B_LEFT_PATH1");
    AutoTrajectory B_LEFT_PATH2 = autoBLeft.trajectory("B_LEFT_PATH2");

    autoBLeft
        .active()
        .onTrue(
            Commands.sequence(
                B_LEFT_PATH1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                B_LEFT_PATH1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    B_LEFT_PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW)));

    return autoBLeft;
  }

  public static final AutoRoutine autoBRight(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoBRight = drive.getAutoFactory().newRoutine("autoBRight");

    AutoTrajectory B_RIGHT_PATH1 = autoBRight.trajectory("B_RIGHT_PATH1");
    AutoTrajectory B_RIGHT_PATH2 = autoBRight.trajectory("B_RIGHT_PATH2");

    autoBRight
        .active()
        .onTrue(
            Commands.sequence(
                B_RIGHT_PATH1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                B_RIGHT_PATH1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    B_RIGHT_PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW)));

    return autoBRight;
  }
}
