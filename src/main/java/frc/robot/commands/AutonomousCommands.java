package frc.robot.commands;

import choreo.Choreo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.v0_funky.kitbot_roller.V0_FunkyRoller;

public class AutonomousCommands {
  public static final Command CharlotteTest(Drive drive) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory("LEFT_SCORE_PRELOADED_OPP")
                        .get()
                        .getInitialPose(true)
                        .get())),
        drive.getAutoFactory().trajectoryCmd("LEFT_SCORE_PRELOADED_OPP"),
        Commands.run(()->roller.runRoller(()->1,()->0)).withTimeout(1),
        drive.getAutoFactory().trajectoryCmd("LEFT_COLLECT_CORAL_OPP"),
        drive.getAutoFactory().trajectoryCmd("LEFT_SCORE_COLLECTED_OPP"),
        Commands.run(()->roller.runRoller(()->1,()->0)).withTimeout(1));
  }
      public static final Command blueLeft2PieceAuto(Drive drive, V0_FunkyRoller roller) {
                    () ->
                RobotState.resetRobotPose(
                    Choreo.loadTrajectory(Choreo.loadTrajectory("MiddleToReef").get().getInitialPose(true).get())),
        drive.getAutoFactory().trajectoryCmd("MiddleToReef"),
        drive.getAutoFactory().trajectoryCmd("ReefToCollect"),
        drive.getAutoFactory().trajectoryCmd("CollectToReef"));
  }
}
