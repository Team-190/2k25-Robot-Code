package frc.robot.commands;

import choreo.Choreo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.v0_funky.kitbot_roller.V0_FunkyRoller;

public class AutonomousCommands {

  public static final Command charlotteTest(Drive drive, V0_FunkyRoller roller) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("MiddleToReef").get().getInitialPose(true).get())),
        drive.getAutoFactory().trajectoryCmd("MiddleToReef"),
        drive.getAutoFactory().trajectoryCmd("ReefToCollect"),
        drive.getAutoFactory().trajectoryCmd("CollectToReef"));
  }

  public static final Command blueLeft3PieceWithRightSource(Drive drive, V0_FunkyRoller roller) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("LEFT_3_CORALS_RIGHT_STATION_Part1")
                        .get()
                        .getInitialPose(true)
                        .get())),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_RIGHT_STATION_Part1"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(2),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_RIGHT_STATION_Part2"),
        Commands.waitSeconds(3),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_RIGHT_STATION_Part3"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(1),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_RIGHT_STATION_Part4"),
        Commands.waitSeconds(3),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_RIGHT_STATION_Part5"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(1));
  }

  public static final Command blueLeft3PieceWithLeftSource(Drive drive, V0_FunkyRoller roller) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("LEFT_3_CORALS_LEFT_STATION_Part1")
                        .get()
                        .getInitialPose(true)
                        .get())),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_LEFT_STATION_Part1"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(2),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_LEFT_STATION_Part2"),
        Commands.waitSeconds(3),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_LEFT_STATION_Part3"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(1),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_LEFT_STATION_Part4"),
        Commands.waitSeconds(3),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_LEFT_STATION_Part5"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(1));
  }

  public static final Command blueLeft2PieceWithLeftSource(Drive drive, V0_FunkyRoller roller) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("LEFT_3_CORALS_LEFT_STATION_Part1")
                        .get()
                        .getInitialPose(true)
                        .get())),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_LEFT_STATION_Part1"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(2),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_LEFT_STATION_Part2"),
        Commands.waitSeconds(3),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_LEFT_STATION_Part3"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(1));
  }

  public static final Command blueLeft2PieceWithRightSource(Drive drive, V0_FunkyRoller roller) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("LEFT_3_CORALS_RIGHT_STATION_Part1")
                        .get()
                        .getInitialPose(true)
                        .get())),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_RIGHT_STATION_Part1"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(2),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_RIGHT_STATION_Part2"),
        Commands.waitSeconds(3),
        drive.getAutoFactory().trajectoryCmd("LEFT_3_CORALS_RIGHT_STATION_Part3"),
        roller.runRoller(() -> 1, () -> 0).withTimeout(1));
  }

  // Run the testPath trajectory 100 times each, one round with velocity and once with twerk drive
  public static final Command testDrive(Drive drive) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("testPath").get().getInitialPose(false).get())),
        drive.getAutoFactory().trajectoryCmd("testPath"));
  }
}
