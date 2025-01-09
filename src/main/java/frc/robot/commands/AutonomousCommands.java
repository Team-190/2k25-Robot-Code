package frc.robot.commands;

import choreo.Choreo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;

public class AutonomousCommands {
  public static final Command blueBarge5PieceAuto(Drive drive) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("BB1").get().getInitialPose(true).get())),
        drive.getAutoFactory().trajectoryCmd("BB1"),
        drive.getAutoFactory().trajectoryCmd("BB2"),
        drive.getAutoFactory().trajectoryCmd("BB3"),
        drive.getAutoFactory().trajectoryCmd("BB4"),
        drive.getAutoFactory().trajectoryCmd("BB5"),
        drive.getAutoFactory().trajectoryCmd("BB6"));
  }
}
