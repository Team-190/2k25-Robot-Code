package frc.robot.commands;

import choreo.Choreo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;

public class AutonomousCommands {
  public static final Command CharlotteTest(Drive drive) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("MiddleToReef").get().getInitialPose(true).get())),
        drive.getAutoFactory().trajectoryCmd("MiddleToReef"),
        drive.getAutoFactory().trajectoryCmd("ReefToCollect"),
        drive.getAutoFactory().trajectoryCmd("CollectToReef"));
  }
}
