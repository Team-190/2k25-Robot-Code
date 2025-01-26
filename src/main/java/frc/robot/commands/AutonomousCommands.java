package frc.robot.commands;

import choreo.Choreo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class AutonomousCommands {
  public static final Command test(Drive drive) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    AllianceFlipUtil.apply(
                        Choreo.loadTrajectory("New Path").get().getInitialPose(true).get()))),
        drive.getAutoFactory().trajectoryCmd("New Path"));
  }
}
