package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_gamma.elevator.Elevator;
import frc.robot.util.AllianceFlipUtil;

public class CompositeCommands {
  // Shared
  public static final Command resetHeading(Drive drive) {
    return Commands.runOnce(
            () -> {
              RobotState.resetRobotPose(
                  new Pose2d(
                      RobotState.getRobotPose().getTranslation(),
                      AllianceFlipUtil.apply(new Rotation2d())));
            })
        .ignoringDisable(true);
  }

  public static final Command scoreCoral(Elevator elevator, String manipulator, int level) {
    return Commands.sequence(
        Commands.print("Move elevator to L" + level),
        Commands.print(manipulator + " score coral")
       );
  }

  public static final Command clapCoral(Elevator elevator, String manipulator, String funnel) {
    return Commands.sequence(
        Commands.print("Elevator to Intake Level"),
        Commands.print(funnel + " run roller"),
        Commands.print(funnel + " clap coral with Clap Daddy TM"),
        Commands.print(funnel + " stop roller"),
        Commands.print("run" + manipulator + " to score coral")
       );
  }

   public static final Command removeAlgae(Elevator elevator, String algaeClapper, int level) {
    return Commands.sequence(
        Commands.print("Move elevator to L" + level),
        Commands.print( algaeClapper + " remove algae")
       );
  }

  public static final Command climb(Elevator elevator, String funnel, String manipulator, String climber) {
    return Commands.sequence(
        Commands.print("Move elevator to STOW"),
        Commands.print(funnel + " run roller to eject coral").onlyIf(()->true), //hasCoral
        Commands.print(funnel + " reverse clapdaddy"),
        Commands.print(climber + " climb")      
       );
  }

  public static final Command autoScoreCoral(Elevator elevator, String manipulator, Drive drive, String reefPost, int level, Camera... cameras) {
    return Commands.sequence(
        Commands.print("DriveCommands.alignRobotToAprilTag(drive, reefPost, cameras)").withTimeout(5),
        scoreCoral(elevator, manipulator, level)
       );
  }
}
