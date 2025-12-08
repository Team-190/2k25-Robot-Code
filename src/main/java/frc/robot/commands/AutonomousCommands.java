package frc.robot.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands.V1_StackUpCompositeCommands;
import frc.robot.commands.CompositeCommands.V2_RedundancyCompositeCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorCSB;
import frc.robot.subsystems.shared.funnel.Funnel.FunnelCSB;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v3_Poot.superstructure.V3_PootSuperstructure;
import frc.robot.subsystems.v3_Poot.superstructure.V3_PootSuperstructureStates;
import frc.robot.subsystems.v3_Poot.superstructure.intake.V3_PootIntake;
import frc.robot.subsystems.v3_Poot.superstructure.manipulator.V3_PootManipulator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeometryUtil;
import frc.robot.util.LoggedChoreo.LoggedAutoRoutine;
import frc.robot.util.LoggedChoreo.LoggedAutoTrajectory;
import java.util.Optional;

public class AutonomousCommands {
  private static Optional<Trajectory<SwerveSample>> A_LEFT_PATH1;
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

    drive.getAutoFactory().cache().loadTrajectory("E_LEFT_PATH_1");
    drive.getAutoFactory().cache().loadTrajectory("E_LEFT_PATH_2");
    drive.getAutoFactory().cache().loadTrajectory("E_LEFT_PATH_3");
    drive.getAutoFactory().cache().loadTrajectory("E_LEFT_PATH_4");
    drive.getAutoFactory().cache().loadTrajectory("E_LEFT_PATH_5");
    drive.getAutoFactory().cache().loadTrajectory("E_LEFT_PATH_6");
    drive.getAutoFactory().cache().loadTrajectory("E_LEFT_PATH_7");

    drive.getAutoFactory().cache().loadTrajectory("F_BENCHMARK_PATH_1_NEW");
    drive.getAutoFactory().cache().loadTrajectory("F_BENCHMARK_PATH_2_NEW");
    drive.getAutoFactory().cache().loadTrajectory("F_BENCHMARK_PATH_3_NEW");
    drive.getAutoFactory().cache().loadTrajectory("F_BENCHMARK_PATH_4_NEW");
    drive.getAutoFactory().cache().loadTrajectory("F_BENCHMARK_PATH_5");
  }

  public static final Command autoALeft(
      Drive drive,
      ElevatorCSB elevator,
      FunnelCSB funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    A_LEFT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        A_LEFT_PATH1_CMD,
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            A_LEFT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            A_LEFT_PATH3_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            A_LEFT_PATH4_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5));
  }

  public static final Command autoARight(
      Drive drive,
      ElevatorCSB elevator,
      FunnelCSB funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    A_RIGHT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
        A_RIGHT_PATH1_CMD,
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            A_RIGHT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            A_RIGHT_PATH3_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            A_RIGHT_PATH4_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.25));
  }

  public static final Command autoBLeft(
      Drive drive,
      ElevatorCSB elevator,
      FunnelCSB funnel,
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
            elevator.setPosition(() -> ReefState.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            B_LEFT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            elevator.setPosition(() -> ReefState.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefState.STOW));
  }

  public static final Command autoCLeft(
      Drive drive,
      ElevatorCSB elevator,
      FunnelCSB funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    C_LEFT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        C_LEFT_PATH1_CMD,
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            C_LEFT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            C_LEFT_PATH3_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        V1_StackUpCompositeCommands.twerk(drive, elevator, manipulator, cameras));
  }

  public static final Command autoCRight(
      Drive drive,
      ElevatorCSB elevator,
      FunnelCSB funnel,
      V1_StackUpManipulator manipulator,
      Camera... cameras) {

    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    C_RIGHT_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
        C_RIGHT_PATH1_CMD,
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            C_RIGHT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            C_RIGHT_PATH3_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
        elevator.setPosition(() -> ReefState.L4),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras), Commands.waitUntil(elevator::atGoal)),
        manipulator.scoreCoral().withTimeout(0.5),
        V1_StackUpCompositeCommands.twerk(drive, elevator, manipulator, cameras));
  }

  public static final Command autoBRight(
      Drive drive,
      ElevatorCSB elevator,
      FunnelCSB funnel,
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
            elevator.setPosition(() -> ReefState.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefState.STOW),
        Commands.deadline(
            B_RIGHT_PATH2_CMD,
            V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator),
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            elevator.setPosition(() -> ReefState.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        elevator.setPosition(() -> ReefState.STOW));
  }

  public static final Command autoDCenter(
      Drive drive, ElevatorCSB elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    D_CENTER_PATH1.get().getInitialPose(AllianceFlipUtil.shouldFlip()).get())),
        Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
        D_CENTER_PATH1_CMD,
        Commands.parallel(
            DriveCommands.autoAlignReefCoral(drive, cameras),
            elevator.setPosition(() -> ReefState.L4)),
        manipulator.scoreCoral().withTimeout(0.5),
        V1_StackUpCompositeCommands.twerk(drive, elevator, manipulator, cameras));
  }

  // V2

  public static final LoggedAutoRoutine autoALeft(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeft");

    LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_LEFT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_LEFT_PATH4")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoALeftNashoba(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeftNashoba");

    LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_LEFT_PATH_ALT3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_LEFT_PATH_ALT4")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoALeftDAVE(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeftD.A.V.E.");

    LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_LEFT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_LEFT_PATH4_ALT_ALT")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.25),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.STOW_DOWN,
                    V2_RedundancySuperstructureStates.SCORE_L4,
                    0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoARight(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoARight");

    LoggedAutoTrajectory path1 = routine.trajectory("A_RIGHT_PATH1");
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("A_RIGHT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("A_RIGHT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path4 =
        routine
            .trajectory("A_RIGHT_PATH4")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.25),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.25),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.25),
                Commands.deadline(
                    path4.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoBLeft(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoBLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("B_LEFT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("B_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCLeft(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_LEFT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_LEFT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCLeftPush(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCLeft");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_LEFT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_LEFT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_LEFT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

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
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));

    return routine;
  }

  public static final LoggedAutoRoutine autoCRight(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_RIGHT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_RIGHT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_RIGHT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));
    return routine;
  }

  public static final LoggedAutoRoutine autoCRightPush(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("C_RIGHT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("C_RIGHT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path3 =
        routine
            .trajectory("C_RIGHT_PATH3")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
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
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path3.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
                superstructure.runGoal(V2_RedundancySuperstructureStates.L4),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    Commands.waitUntil(() -> superstructure.atGoal())),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));
    return routine;
  }

  public static final LoggedAutoRoutine autoBRight(
      Drive drive,
      V2_RedundancyIntake intake,
      V2_RedundancySuperstructure superstructure,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoBRight");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("B_RIGHT_PATH1")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));
    LoggedAutoTrajectory path2 =
        routine
            .trajectory("B_RIGHT_PATH2")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                Commands.deadline(
                    path2.cmd(),
                    V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                        superstructure, intake),
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5),
                superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN)));

    return routine;
  }

  public static final LoggedAutoRoutine autoDCenter(
      Drive drive, V2_RedundancySuperstructure superstructure, Camera... cameras) {
    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoDCenter");
    LoggedAutoTrajectory path1 =
        routine
            .trajectory("D_CENTER_PATH")
            .bindEvent("Funnel", Commands.runOnce(() -> RobotState.setAutoClapOverride(true)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                Commands.parallel(
                    DriveCommands.autoAlignReefCoral(drive, cameras),
                    superstructure.runGoal(V2_RedundancySuperstructureStates.L4)),
                superstructure.runActionWithTimeout(
                    V2_RedundancySuperstructureStates.SCORE_L4, 0.5)));
    return routine;
  }

  // V3

  //     public static final LoggedAutoRoutine autoALeft(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {
  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeft");

  //       LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("A_LEFT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path3 =
  //           routine
  //               .trajectory("A_LEFT_PATH3")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path4 =
  //           routine
  //               .trajectory("A_LEFT_PATH4")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
  //                   path1.cmd(),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path3.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path4.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));

  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoALeftNashoba(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {
  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeftNashoba");

  //       LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("A_LEFT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path3 =
  //           routine
  //               .trajectory("A_LEFT_PATH_ALT3")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path4 =
  //           routine
  //               .trajectory("A_LEFT_PATH_ALT4")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
  //                   path1.cmd(),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path3.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path4.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));

  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoALeftDAVE(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {
  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoALeftD.A.V.E.");

  //       LoggedAutoTrajectory path1 = routine.trajectory("A_LEFT_PATH1");
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("A_LEFT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path3 =
  //           routine
  //               .trajectory("A_LEFT_PATH3")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path4 =
  //           routine
  //               .trajectory("A_LEFT_PATH4_ALT_ALT")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
  //                   path1.cmd(),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path3.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path4.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));

  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoARight(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {

  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoARight");

  //       LoggedAutoTrajectory path1 = routine.trajectory("A_RIGHT_PATH1");
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("A_RIGHT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path3 =
  //           routine
  //               .trajectory("A_RIGHT_PATH3")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path4 =
  //           routine
  //               .trajectory("A_RIGHT_PATH4")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
  //                   path1.cmd(),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path3.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.25),
  //                   Commands.deadline(
  //                       path4.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));

  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoBLeft(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {

  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoBLeft");
  //       LoggedAutoTrajectory path1 =
  //           routine
  //               .trajectory("B_LEFT_PATH1")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("B_LEFT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
  //                   path1.cmd(),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE)),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       superstructure.runGoal(V3_PootSuperstructureStates.L4)),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.STOW_DOWN)));

  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoCLeft(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {

  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCLeft");
  //       LoggedAutoTrajectory path1 =
  //           routine
  //               .trajectory("C_LEFT_PATH1")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("C_LEFT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path3 =
  //           routine
  //               .trajectory("C_LEFT_PATH3")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
  //                   path1.cmd(),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path3.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));

  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoCLeftPush(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {

  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCLeft");
  //       LoggedAutoTrajectory path1 =
  //           routine
  //               .trajectory("C_LEFT_PATH1")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("C_LEFT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path3 =
  //           routine
  //               .trajectory("C_LEFT_PATH3")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runEnd(
  //                           () -> drive.runVelocity(new ChassisSpeeds(0.0, -1.0, 0.0)),
  //                           () -> drive.stop())
  //                       .withTimeout(0.5),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
  //                   path1.cmd(),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path3.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));

  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoCRight(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {

  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCRight");
  //       LoggedAutoTrajectory path1 =
  //           routine
  //               .trajectory("C_RIGHT_PATH1")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("C_RIGHT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path3 =
  //           routine
  //               .trajectory("C_RIGHT_PATH3")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
  //                   path1.cmd(),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path3.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));
  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoCRightPush(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {

  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoCRight");
  //       LoggedAutoTrajectory path1 =
  //           routine
  //               .trajectory("C_RIGHT_PATH1")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("C_RIGHT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path3 =
  //           routine
  //               .trajectory("C_RIGHT_PATH3")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
  //                   Commands.runEnd(
  //                           () -> drive.runVelocity(new ChassisSpeeds(0.0, 1.0, 0.0)),
  //                           () -> drive.stop())
  //                       .withTimeout(0.5),
  //                   path1.cmd(),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path3.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.runOnce(() -> RobotState.setAutoClapOverride(false)),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       Commands.waitUntil(() -> superstructure.atGoal())),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));
  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoBRight(
  //         Drive drive,
  //         V3_PootIntake intake,
  //         V3_PootSuperstructure superstructure,
  //         V3_PootManipulator manipulator,
  //         Camera... cameras) {

  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoBRight");
  //       LoggedAutoTrajectory path1 =
  //           routine
  //               .trajectory("B_RIGHT_PATH1")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));
  //       LoggedAutoTrajectory path2 =
  //           routine
  //               .trajectory("B_RIGHT_PATH2")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
  //                   path1.cmd(),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       superstructure.runGoal(V3_PootSuperstructureStates.L4)),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   Commands.deadline(
  //                       path2.cmd(),
  //                       CompositeCommands.V3_PootCompositeCommands.intakeCoralDriverSequence(
  //                           superstructure, intake, manipulator),
  //                       Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT))),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE)),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5),
  //                   superstructure.runGoal(V3_PootSuperstructureStates.STOW_DOWN)));

  //       return routine;
  //     }

  //     public static final LoggedAutoRoutine autoDCenter(
  //         Drive drive, V3_PootSuperstructure superstructure, Camera... cameras) {
  //       LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoDCenter");
  //       LoggedAutoTrajectory path1 =
  //           routine
  //               .trajectory("D_CENTER_PATH")
  //               .bindEvent("Funnel", Commands.runOnce(() ->
  // RobotState.setAutoClapOverride(true)));

  //       routine
  //           .active()
  //           .onTrue(
  //               Commands.sequence(
  //                   path1.resetOdometry(),
  //                   Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
  //                   path1.cmd(),
  //                   Commands.parallel(
  //                       DriveCommands.autoAlignReefCoral(drive, cameras),
  //                       superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE)),
  //                   superstructure.runActionWithTimeout(
  //                       V3_PootSuperstructureStates.STOW_DOWN,
  //                       V3_PootSuperstructureStates.L4_SCORE,
  //                       0.5)));
  //       return routine;
  //    }

  public static final LoggedAutoRoutine autoERight(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoERight");
    LoggedAutoTrajectory path1 = routine.trajectory("E_RIGHT_PATH_1");
    LoggedAutoTrajectory path2 = routine.trajectory("E_RIGHT_PATH_2");
    LoggedAutoTrajectory path3 = routine.trajectory("E_RIGHT_PATH_3");
    LoggedAutoTrajectory path4 = routine.trajectory("E_RIGHT_PATH_4");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                Commands.parallel(
                    path1.cmd(), superstructure.runGoal(V3_PootSuperstructureStates.STOW_UP)),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                    path2.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.5)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L4))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                    path3.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.5)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L4))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                    path4.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.5)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L4))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE)));

    return routine;
  }

  public static final LoggedAutoRoutine autoELeft(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoELeft");
    LoggedAutoTrajectory path1 =
        routine.trajectory(GeometryUtil.mirrorTrajectory(routine.trajectory("E_RIGHT_PATH_1")));
    LoggedAutoTrajectory path2 =
        routine.trajectory(GeometryUtil.mirrorTrajectory(routine.trajectory("E_RIGHT_PATH_2")));
    LoggedAutoTrajectory path3 =
        routine.trajectory(GeometryUtil.mirrorTrajectory(routine.trajectory("E_RIGHT_PATH_3")));
    LoggedAutoTrajectory path4 =
        routine.trajectory(GeometryUtil.mirrorTrajectory(routine.trajectory("E_RIGHT_PATH_4")));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                    path2.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((2.0)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.STOW_UP))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                    path3.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.5)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.STOW_UP))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                    path4.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.5)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.STOW_UP))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE)));

    return routine;
  }

  public static final LoggedAutoRoutine autoERightBack(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoERightBack");
    LoggedAutoTrajectory path1 = routine.trajectory("E_RIGHT_PATH_1_BACK");
    LoggedAutoTrajectory path2 = routine.trajectory("E_RIGHT_PATH_2_BACK");
    LoggedAutoTrajectory path3 = routine.trajectory("E_RIGHT_PATH_3_BACK");
    LoggedAutoTrajectory path4 = routine.trajectory("E_RIGHT_PATH_4_BACK");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                path1.cmd(),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    path2.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.0)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L2))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L2, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L2_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                    path3.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.0)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L2))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L2, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L2_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    path4.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.0)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L4))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE)));

    return routine;
  }

  public static final LoggedAutoRoutine autoELeftBack(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoELeftBack");
    LoggedAutoTrajectory path1 =
        routine.trajectory(
            GeometryUtil.mirrorTrajectory(routine.trajectory("E_RIGHT_PATH_1_BACK")));
    LoggedAutoTrajectory path2 =
        routine.trajectory(
            GeometryUtil.mirrorTrajectory(routine.trajectory("E_RIGHT_PATH_2_BACK")));
    LoggedAutoTrajectory path3 =
        routine.trajectory(
            GeometryUtil.mirrorTrajectory(routine.trajectory("E_RIGHT_PATH_3_BACK")));
    LoggedAutoTrajectory path4 =
        routine.trajectory(
            GeometryUtil.mirrorTrajectory(routine.trajectory("E_RIGHT_PATH_4_BACK")));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)),
                path1.cmd(),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    path2.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.0)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L2))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L2, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L2_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                    path3.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.0)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L2))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L2, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L2_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    path4.cmd(),
                    superstructure
                        .runGoal(V3_PootSuperstructureStates.GROUND_INTAKE_CORAL)
                        .andThen(Commands.waitSeconds((1.0)))
                        .andThen(superstructure.runGoal(V3_PootSuperstructureStates.L4))),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE)));

    return routine;
  }

  public static final LoggedAutoRoutine autoFLeft(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoFLeft");
    LoggedAutoTrajectory path1 = (routine.trajectory("F_PATH_1"));
    LoggedAutoTrajectory path2 = (routine.trajectory("F_PATH_2"));
    LoggedAutoTrajectory path3 = (routine.trajectory("F_PATH_3"));
    LoggedAutoTrajectory path4 = (routine.trajectory("F_PATH_4"));
    LoggedAutoTrajectory path5 = (routine.trajectory("F_PATH_5"));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.parallel(
                    path2.cmd(),
                    superstructure.runGoal(V3_PootSuperstructureStates.L3_ALGAE_INTAKE)),
                Commands.parallel(
                    path3.cmd(), superstructure.runGoal(V3_PootSuperstructureStates.STOW_UP)),
                Commands.runOnce(() -> drive.stop()),
                CompositeCommands.V3_PootCompositeCommands.optimalScoreBarge(superstructure),
                Commands.parallel(
                    path4.cmd(),
                    superstructure.runGoal(V3_PootSuperstructureStates.L2_ALGAE_INTAKE)),
                Commands.parallel(
                    path5.cmd(), superstructure.runGoal(V3_PootSuperstructureStates.STOW_UP)),
                Commands.runOnce(() -> drive.stop()),
                CompositeCommands.V3_PootCompositeCommands.optimalScoreBarge(superstructure)));

    return routine;
  }

  public static final LoggedAutoRoutine autoFLeftMinimal(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("autoFLeftMinimal");
    LoggedAutoTrajectory path1 = (routine.trajectory("F_PATH_1"));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)),
                CompositeCommands.V3_PootCompositeCommands.optimalAutoScoreCoralSequence(
                    drive, superstructure, ReefState.L4, cameras),
                superstructure.runGoal(V3_PootSuperstructureStates.L4_SCORE),
                superstructure.waitUntilAtGoal(),
                Commands.runOnce(() -> drive.stop())));

    return routine;
  }

  public static final LoggedAutoRoutine accuracyBenchOne(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("accuracyBench");
    LoggedAutoTrajectory path1 = (routine.trajectory("F_BENCHMARK_PATH_1_NEW"));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(), path1.cmd(), Commands.runOnce(() -> drive.stop())));

    return routine;
  }

  public static final LoggedAutoRoutine accuracyBenchTwo(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("accuracyBench");
    LoggedAutoTrajectory path1 = (routine.trajectory("F_BENCHMARK_PATH_2_NEW"));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(), path1.cmd(), Commands.runOnce(() -> drive.stop())));

    return routine;
  }

  public static final LoggedAutoRoutine accuracyBenchThree(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("accuracyBench");
    LoggedAutoTrajectory path1 = (routine.trajectory("F_BENCHMARK_PATH_3_NEW"));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(), path1.cmd(), Commands.runOnce(() -> drive.stop())));

    return routine;
  }

  public static final LoggedAutoRoutine accuracyBenchFour(
      Drive drive,
      V3_PootSuperstructure superstructure,
      V3_PootIntake intake,
      V3_PootManipulator manipulator,
      Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("accuracyBench");
    LoggedAutoTrajectory path1 = (routine.trajectory("F_BENCHMARK_PATH_4_NEW"));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(), path1.cmd(), Commands.runOnce(() -> drive.stop())));

    return routine;
  }

  public static final LoggedAutoRoutine accuracyBenchFive(Drive drive, Camera... cameras) {

    LoggedAutoRoutine routine = drive.getAutoFactory().newRoutine("accuracyBench");
    LoggedAutoTrajectory path1 = (routine.trajectory("F_BENCHMARK_PATH_5"));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                path1.resetOdometry(), path1.cmd(), Commands.runOnce(() -> drive.stop())));

    return routine;
  }
}
