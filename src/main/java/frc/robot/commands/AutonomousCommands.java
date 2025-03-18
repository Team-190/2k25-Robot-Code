package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands.AlgaeCommands;
import frc.robot.commands.CompositeCommands.IntakeCommands;
import frc.robot.commands.CompositeCommands.ScoreCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.shared.funnel.Funnel;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;

public class AutonomousCommands {
  private static Command A_LEFT_PATH1;
  private static Command A_LEFT_PATH2;
  private static Command A_LEFT_PATH3;
  private static Command A_LEFT_PATH4;

  private static Command A_RIGHT_PATH1;
  private static Command A_RIGHT_PATH2;
  private static Command A_RIGHT_PATH3;
  private static Command A_RIGHT_PATH4;

  private static Command B_LEFT_PATH1;
  private static Command B_LEFT_PATH2;

  private static Command B_RIGHT_PATH1;
  private static Command B_RIGHT_PATH2;

  private static Command C_LEFT_PATH1;
  private static Command C_LEFT_PATH2;
  private static Command C_LEFT_PATH3;

  private static Command C_RIGHT_PATH1;
  private static Command C_RIGHT_PATH2;
  private static Command C_RIGHT_PATH3;

  private static Command D_CENTER_PATH1;

  public static void loadAutoTrajectories(Drive drive) {
    A_LEFT_PATH1 = drive.getAutoFactory().trajectoryCmd("A_LEFT_PATH1");
    A_LEFT_PATH2 = drive.getAutoFactory().trajectoryCmd("A_LEFT_PATH2");
    A_LEFT_PATH3 = drive.getAutoFactory().trajectoryCmd("A_LEFT_PATH3");
    A_LEFT_PATH4 = drive.getAutoFactory().trajectoryCmd("A_LEFT_PATH4");

    A_RIGHT_PATH1 = drive.getAutoFactory().trajectoryCmd("A_RIGHT_PATH1");
    A_RIGHT_PATH2 = drive.getAutoFactory().trajectoryCmd("A_RIGHT_PATH2");
    A_RIGHT_PATH3 = drive.getAutoFactory().trajectoryCmd("A_RIGHT_PATH3");
    A_RIGHT_PATH4 = drive.getAutoFactory().trajectoryCmd("A_RIGHT_PATH4");

    B_LEFT_PATH1 = drive.getAutoFactory().trajectoryCmd("B_LEFT_PATH1");
    B_LEFT_PATH2 = drive.getAutoFactory().trajectoryCmd("B_LEFT_PATH2");

    B_RIGHT_PATH1 = drive.getAutoFactory().trajectoryCmd("B_RIGHT_PATH1");
    B_RIGHT_PATH2 = drive.getAutoFactory().trajectoryCmd("B_RIGHT_PATH2");

    C_LEFT_PATH1 = drive.getAutoFactory().trajectoryCmd("C_LEFT_PATH1");
    C_LEFT_PATH2 = drive.getAutoFactory().trajectoryCmd("C_LEFT_PATH2");
    C_LEFT_PATH3 = drive.getAutoFactory().trajectoryCmd("C_LEFT_PATH3");

    C_RIGHT_PATH1 = drive.getAutoFactory().trajectoryCmd("C_RIGHT_PATH1");
    C_RIGHT_PATH2 = drive.getAutoFactory().trajectoryCmd("C_RIGHT_PATH2");
    C_RIGHT_PATH3 = drive.getAutoFactory().trajectoryCmd("C_RIGHT_PATH3");

    D_CENTER_PATH1 = drive.getAutoFactory().trajectoryCmd("D_CENTER_PATH1");
  }

  public static final Command autoALeft(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        A_LEFT_PATH1,
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            A_LEFT_PATH2,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            A_LEFT_PATH3,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            A_LEFT_PATH4,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5));
  }

  public static final Command autoARight(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
        A_RIGHT_PATH1,
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            A_RIGHT_PATH2,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            A_RIGHT_PATH3,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            A_RIGHT_PATH4,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25));
  }

  public static final Command autoBLeft(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
        B_LEFT_PATH1,
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), elevator.setPosition(ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            B_LEFT_PATH2,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), elevator.setPosition(ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(ReefHeight.STOW));
  }

  public static final Command autoCLeft(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        C_LEFT_PATH1,
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            C_LEFT_PATH2,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            C_LEFT_PATH3,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        AlgaeCommands.twerk(drive, elevator, manipulator, cameras));
  }

  public static final Command autoCRight(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
        C_RIGHT_PATH1,
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            C_RIGHT_PATH2,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            C_RIGHT_PATH3,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        AlgaeCommands.twerk(drive, elevator, manipulator, cameras));
  }

  public static final Command autoBRight(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        B_RIGHT_PATH1,
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), elevator.setPosition(ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(ReefHeight.STOW),
        Commands.deadline(
            B_RIGHT_PATH2,
            IntakeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), elevator.setPosition(ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(ReefHeight.STOW));
  }

  public static final Command autoDCenter(
      Drive drive, Elevator elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
    return Commands.sequence(
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        D_CENTER_PATH1,
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), elevator.setPosition(ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        AlgaeCommands.twerk(drive, elevator, manipulator, cameras));
  }
}
