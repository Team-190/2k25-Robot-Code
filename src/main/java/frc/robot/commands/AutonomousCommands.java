package frc.robot.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands.SharedCommands.V1_StackUpCompositeCommands;
import frc.robot.commands.CompositeCommands.SharedCommands.V2_RedundancyCompositeCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.shared.funnel.Funnel;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedChoreo.LoggedAutoRoutine;
import frc.robot.util.LoggedChoreo.LoggedAutoTrajectory;
import java.util.Optional;

public class AutonomousCommands {
  private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH1;
  private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH2;
  private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH3;
  private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH4;

  private static Optional<Trajectory<SwerveSample>> A_RIGHT_PATH1;
  private static Optional<Trajectory<SwerveSample>> B_LEFT_PATH1;
  private static Optional<Trajectory<SwerveSample>> B_RIGHT_PATH1;
  private static Optional<Trajectory<SwerveSample>> C_LEFT_PATH1;
  private static Optional<Trajectory<SwerveSample>> C_RIGHT_PATH1;
  private static Optional<Trajectory<SwerveSample>> D_CENTER_PATH1;

  private static Command A_LEFT_PATH1_CMD;
  private static Command A_LEFT_PATH2_CMD;
  private static Command A_LEFT_PATH3_CMD;
  private static Command A_LEFT_PATH4_CMD;

  private static Command A_RIGHT_PATH1_CMD;
  private static Command A_RIGHT_PATH2_CMD;
  private static Command A_RIGHT_PATH3_CMD;
  private static Command A_RIGHT_PATH4_CMD;

  private static Command B_LEFT_PATH1_CMD;
  private static Command B_LEFT_PATH2_CMD;

  private static Command B_RIGHT_PATH1_CMD;
  private static Command B_RIGHT_PATH2_CMD;

  private static Command C_LEFT_PATH1_CMD;
  private static Command C_LEFT_PATH2_CMD;
  private static Command C_LEFT_PATH3_CMD;

  private static Command C_RIGHT_PATH1_CMD;
  private static Command C_RIGHT_PATH2_CMD;
  private static Command C_RIGHT_PATH3_CMD;

  private static Command D_CENTER_PATH1_CMD;

  static {
  }

  public static void loadAutoTrajectoriesOld(Drive drive) {
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH3");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH4");

    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH3");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH4");

    drive.getAutoFactory().cache().loadTrajectory("B_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("B_LEFT_PATH2");

    drive.getAutoFactory().cache().loadTrajectory("B_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("B_RIGHT_PATH2");

    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH3");

    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH3");

    drive.getAutoFactory().cache().loadTrajectory("D_CENTER_PATH1");

    A_LEFT_PATH1 = Choreo.loadTrajectory("A_LEFT_PATH1");
    A_RIGHT_PATH1 = Choreo.loadTrajectory("A_RIGHT_PATH1");
    B_LEFT_PATH1 = Choreo.loadTrajectory("B_LEFT_PATH1");
    B_RIGHT_PATH1 = Choreo.loadTrajectory("B_RIGHT_PATH1");
    C_LEFT_PATH1 = Choreo.loadTrajectory("C_LEFT_PATH1");
    C_RIGHT_PATH1 = Choreo.loadTrajectory("C_RIGHT_PATH1");
    D_CENTER_PATH1 = Choreo.loadTrajectory("D_CENTER_PATH");

    A_LEFT_PATH1_CMD = drive.getAutoFactory().trajectoryCmd("A_LEFT_PATH1");
    A_LEFT_PATH2_CMD = drive.getAutoFactory().trajectoryCmd("A_LEFT_PATH2");
    A_LEFT_PATH3_CMD = drive.getAutoFactory().trajectoryCmd("A_LEFT_PATH3");
    A_LEFT_PATH4_CMD = drive.getAutoFactory().trajectoryCmd("A_LEFT_PATH4");

    A_RIGHT_PATH1_CMD = drive.getAutoFactory().trajectoryCmd("A_RIGHT_PATH1");
    A_RIGHT_PATH2_CMD = drive.getAutoFactory().trajectoryCmd("A_RIGHT_PATH2");
    A_RIGHT_PATH3_CMD = drive.getAutoFactory().trajectoryCmd("A_RIGHT_PATH3");
    A_RIGHT_PATH4_CMD = drive.getAutoFactory().trajectoryCmd("A_RIGHT_PATH4");

    B_LEFT_PATH1_CMD = drive.getAutoFactory().trajectoryCmd("B_LEFT_PATH1");
    B_LEFT_PATH2_CMD = drive.getAutoFactory().trajectoryCmd("B_LEFT_PATH2");

    B_RIGHT_PATH1_CMD = drive.getAutoFactory().trajectoryCmd("B_RIGHT_PATH1");
    B_RIGHT_PATH2_CMD = drive.getAutoFactory().trajectoryCmd("B_RIGHT_PATH2");

    C_LEFT_PATH1_CMD = drive.getAutoFactory().trajectoryCmd("C_LEFT_PATH1");
    C_LEFT_PATH2_CMD = drive.getAutoFactory().trajectoryCmd("C_LEFT_PATH2");
    C_LEFT_PATH3_CMD = drive.getAutoFactory().trajectoryCmd("C_LEFT_PATH3");

    C_RIGHT_PATH1_CMD = drive.getAutoFactory().trajectoryCmd("C_RIGHT_PATH1");
    C_RIGHT_PATH2_CMD = drive.getAutoFactory().trajectoryCmd("C_RIGHT_PATH2");
    C_RIGHT_PATH3_CMD = drive.getAutoFactory().trajectoryCmd("C_RIGHT_PATH3");

    D_CENTER_PATH1_CMD = drive.getAutoFactory().trajectoryCmd("D_CENTER_PATH");
  }

  public static void loadAutoTrajectories(Drive drive) {

    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH3");
    drive.getAutoFactory().cache().loadTrajectory("A_LEFT_PATH4");

    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH3");
    drive.getAutoFactory().cache().loadTrajectory("A_RIGHT_PATH4");

    drive.getAutoFactory().cache().loadTrajectory("B_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("B_LEFT_PATH2");

    drive.getAutoFactory().cache().loadTrajectory("B_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("B_RIGHT_PATH2");

    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("C_LEFT_PATH3");

    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH1");
    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH2");
    drive.getAutoFactory().cache().loadTrajectory("C_RIGHT_PATH3");

    drive.getAutoFactory().cache().loadTrajectory("D_CENTER_PATH");
  }

  public static final Command autoALeft(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    A_LEFT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        A_LEFT_PATH1_CMD,
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            A_LEFT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            A_LEFT_PATH3_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            A_LEFT_PATH4_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefHeight.L4),
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
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    A_RIGHT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
        A_RIGHT_PATH1_CMD,
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            A_RIGHT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            A_RIGHT_PATH3_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            A_RIGHT_PATH4_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefHeight.L4),
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
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    B_LEFT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
        B_LEFT_PATH1_CMD,
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            elevator.setPosition(() -> ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            B_LEFT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            elevator.setPosition(() -> ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefHeight.STOW));
  }

  public static final Command autoCLeft(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    C_LEFT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        C_LEFT_PATH1_CMD,
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            C_LEFT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            C_LEFT_PATH3_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        V1_StackUpCompositeCommands.twerk(drive, elevator, manipulator, cameras));
  }

  public static final Command autoCRight(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    C_RIGHT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
        C_RIGHT_PATH1_CMD,
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            C_RIGHT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            C_RIGHT_PATH3_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefHeight.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        V1_StackUpCompositeCommands.twerk(drive, elevator, manipulator, cameras));
  }

  public static final Command autoBRight(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    B_RIGHT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        B_RIGHT_PATH1_CMD,
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            elevator.setPosition(() -> ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefHeight.STOW),
        Commands.deadline(
            B_RIGHT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            elevator.setPosition(() -> ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefHeight.STOW));
  }

  public static final Command autoDCenter(
      Drive drive, Elevator elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    D_CENTER_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        D_CENTER_PATH1_CMD,
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            elevator.setPosition(() -> ReefHeight.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        V1_StackUpCompositeCommands.twerk(drive, elevator, manipulator, cameras));
  }

  // V2

  public static final LoggedAutoRoutine autoALeft(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeft");

    LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_LEFT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_LEFT_PATH3")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_LEFT_PATH4")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));

    return routine;
  }


  public static final LoggedAutoRoutine autoALeftAlternate(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeftAlternate");

    LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_LEFT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_LEFT_PATH_ALT3")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_LEFT_PATH_ALT4")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));

    return routine;
  }


  public static final LoggedAutoRoutine autoARight(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoARight");

    LoggedAutoTrajectory path1 = routine.trajectory("A_RIGHT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_RIGHT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_RIGHT_PATH3")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_RIGHT_PATH4")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.25),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoBLeft(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoBLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("B_LEFT_PATH1")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("B_LEFT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(() -> ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(() -> ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCLeft(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_LEFT_PATH1")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_LEFT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_LEFT_PATH3")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCLeftPush(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_LEFT_PATH1")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_LEFT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_LEFT_PATH3")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runEnd(
                        () -> drive.runVelocity(new ChassisSpeeds(0.0, -1.0, 0.0)),
                        () -> drive.stop())
                    .withTimeout(0.5),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCRight(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_RIGHT_PATH1")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_RIGHT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_RIGHT_PATH3")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));
    return routine;
  }

  public static final LoggedAutoRoutine autoCRightPush(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_RIGHT_PATH1")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_RIGHT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_RIGHT_PATH3")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                Commands.runEnd(
                        () -> drive.runVelocity(new ChassisSpeeds(0.0, 1.0, 0.0)),
                        () -> drive.stop())
                    .withTimeout(0.5),
                path1.cmd(),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                elevator.setPosition(() -> ReefHeight.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(elevator::atGoal)),
                manipulator.scoreCoral().withTimeout(0.5)));
    return routine;
  }

  public static final LoggedAutoRoutine autoBRight(
      Drive drive,
      Elevator elevator,
      Funnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoBRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("B_RIGHT_PATH1")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("B_RIGHT_PATH2")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(() -> ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralAuto(
                        elevator, funnel, manipulator, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(() -> ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5),
                elevator.setPosition(() -> ReefHeight.STOW)));

    return routine;
  }

  public static final LoggedAutoRoutine autoDCenter(
      Drive drive,
      Elevator elevator,
      V2_RedundancyManipulator manipulator,
      Funnel funnel,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoDCenter");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("D_CENTER_PATH")
            .bindEvent("Funnel", funnel.setClapDaddyGoal(FunnelState.CLOSED));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    elevator.setPosition(() -> ReefHeight.L4)),
                manipulator.scoreCoral().withTimeout(0.5)));
    return routine;
  }
}
