package frc.robot.commands;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands.IntakeCommands;
import frc.robot.commands.CompositeCommands.ScoreCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.elevator.V1_StackUpElevator;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnel;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;

public class AutonomousCommands {
    private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH1;
    private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH2;
    private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH3;
    private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH4;

    private static Optional<Trajectory<SwerveSample>> A_RIGHT_PATH1;
    private static Optional<Trajectory<SwerveSample>> A_RIGHT_PATH2;
    private static Optional<Trajectory<SwerveSample>> A_RIGHT_PATH3;
    private static Optional<Trajectory<SwerveSample>> A_RIGHT_PATH4;

    private static Optional<Trajectory<SwerveSample>> B_LEFT_PATH1;
    private static Optional<Trajectory<SwerveSample>> B_LEFT_PATH2;

    private static Optional<Trajectory<SwerveSample>> B_RIGHT_PATH1;
    private static Optional<Trajectory<SwerveSample>> B_RIGHT_PATH2;

    private static Optional<Trajectory<SwerveSample>> C_LEFT_PATH1;
    private static Optional<Trajectory<SwerveSample>> C_LEFT_PATH2;
    private static Optional<Trajectory<SwerveSample>> C_LEFT_PATH3;

    private static Optional<Trajectory<SwerveSample>> C_RIGHT_PATH1;
    private static Optional<Trajectory<SwerveSample>> C_RIGHT_PATH2;
    private static Optional<Trajectory<SwerveSample>> C_RIGHT_PATH3;

    private static Optional<Trajectory<SwerveSample>> D_CENTER_PATH1;

    static {
        A_LEFT_PATH1 = Choreo.loadTrajectory("A_LEFT_PATH1");
        A_LEFT_PATH2 = Choreo.loadTrajectory("A_LEFT_PATH2");
        A_LEFT_PATH3 = Choreo.loadTrajectory("A_LEFT_PATH3");
        A_LEFT_PATH4 = Choreo.loadTrajectory("A_LEFT_PATH4");

        A_RIGHT_PATH1 = Choreo.loadTrajectory("A_RIGHT_PATH1");
        A_RIGHT_PATH2 = Choreo.loadTrajectory("A_RIGHT_PATH2");
        A_RIGHT_PATH3 = Choreo.loadTrajectory("A_RIGHT_PATH3");
        A_RIGHT_PATH4 = Choreo.loadTrajectory("A_RIGHT_PATH4");

        B_LEFT_PATH1 = Choreo.loadTrajectory("B_LEFT_PATH1");
        B_LEFT_PATH2 = Choreo.loadTrajectory("B_LEFT_PATH2");

        B_RIGHT_PATH1 = Choreo.loadTrajectory("B_RIGHT_PATH3");
        B_RIGHT_PATH2 = Choreo.loadTrajectory("B_RIGHT_PATH4");

        C_LEFT_PATH1 = Choreo.loadTrajectory("C_LEFT_PATH1");
        C_LEFT_PATH2 = Choreo.loadTrajectory("C_LEFT_PATH2");
        C_LEFT_PATH3 = Choreo.loadTrajectory("C_LEFT_PATH3");

        C_RIGHT_PATH1 = Choreo.loadTrajectory("C_RIGHT_PATH1");
        C_RIGHT_PATH2 = Choreo.loadTrajectory("C_RIGHT_PATH2");
        C_RIGHT_PATH3 = Choreo.loadTrajectory("C_RIGHT_PATH3");

        D_CENTER_PATH1 = Choreo.loadTrajectory("D_CENTER_PATH1");
    }

  public static final AutoRoutine autoALeft(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoALeft = drive.getAutoFactory().newRoutine("autoALeft");

    AutoTrajectory PATH1 = autoALeft.trajectory(A_LEFT_PATH1.get());
    AutoTrajectory PATH2 = autoALeft.trajectory(A_LEFT_PATH2.get());
    AutoTrajectory PATH3 = autoALeft.trajectory(A_LEFT_PATH3.get());
    AutoTrajectory PATH4 = autoALeft.trajectory(A_LEFT_PATH4.get());

    autoALeft
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                PATH1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH4.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
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

    AutoTrajectory PATH1 = autoARight.trajectory(A_RIGHT_PATH1.get());
    AutoTrajectory PATH2 = autoARight.trajectory(A_RIGHT_PATH2.get());
    AutoTrajectory PATH3 = autoARight.trajectory(A_RIGHT_PATH3.get());
    AutoTrajectory PATH4 = autoARight.trajectory(A_RIGHT_PATH4.get());

    autoARight
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                PATH1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH4.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25)));

    return autoARight;
  }

  public static final AutoRoutine autoBLeft(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoBLeft = drive.getAutoFactory().newRoutine("autoBLeft");

    AutoTrajectory PATH1 = autoBLeft.trajectory(B_LEFT_PATH1.get());
    AutoTrajectory PATH2 = autoBLeft.trajectory(B_LEFT_PATH2.get());

    autoBLeft
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                PATH1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW)));

    return autoBLeft;
  }

  public static final AutoRoutine autoCLeft(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoCLeft = drive.getAutoFactory().newRoutine("autoCLeft");

    AutoTrajectory PATH1 = autoCLeft.trajectory(C_LEFT_PATH1.get());
    AutoTrajectory PATH2 = autoCLeft.trajectory(C_LEFT_PATH2.get());
    AutoTrajectory PATH3 = autoCLeft.trajectory(C_LEFT_PATH3.get());

    autoCLeft
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                PATH1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                ScoreCommands.twerk(drive, elevator, manipulator, cameras)));

    return autoCLeft;
  }

  public static final AutoRoutine autoCRight(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoCRight = drive.getAutoFactory().newRoutine("autoCRight");

    AutoTrajectory PATH1 = autoCRight.trajectory(C_RIGHT_PATH1.get());
    AutoTrajectory PATH2 = autoCRight.trajectory(C_RIGHT_PATH2.get());
    AutoTrajectory PATH3 = autoCRight.trajectory(C_RIGHT_PATH3.get());

    autoCRight
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                PATH1.cmd(),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH2.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(ReefHeight.STOW),
                Commands.deadline(
                    PATH3.cmd(),
                    IntakeCommands.intakeCoral(elevator, funnel, manipulator),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                ScoreCommands.twerk(drive, elevator, manipulator, cameras)));

    return autoCRight;
  }

  public static final AutoRoutine autoBRight(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpFunnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoBRight = drive.getAutoFactory().newRoutine("autoBRight");

    AutoTrajectory PATH_1 = autoBRight.trajectory(B_RIGHT_PATH1.get());
    AutoTrajectory PATH_2 = autoBRight.trajectory(B_RIGHT_PATH2.get());

    autoBRight
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
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

    return autoBRight;
  }

  public static final AutoRoutine autoDCenter(
      Drive drive,
      V1_StackUpElevator elevator,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {
    AutoRoutine autoDCenter = drive.getAutoFactory().newRoutine("autoDCenter");

    AutoTrajectory PATH1 = autoDCenter.trajectory(D_CENTER_PATH1.get());

    autoDCenter
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                PATH1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                ScoreCommands.twerk(drive, elevator, manipulator, cameras)));
    return autoDCenter;
  }
}
