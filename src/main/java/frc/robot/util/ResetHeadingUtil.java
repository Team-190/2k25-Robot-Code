package frc.robot.util;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ResetHeadingUtil {
  private static final HashMap<String, AutoTrajectory> resetPoseMap;
  private static AutoTrajectory defaultTrajectory;
  private static boolean resetHeading;
  private static final Timer timer;

  static {
    resetPoseMap = new HashMap<>();
    resetHeading = true;
    timer = new Timer();
  }

  public static Command resetPose(AutoTrajectory trajectory) {
    return trajectory.resetOdometry().ignoringDisable(true);
  }

  public static Command resetPose(LoggedDashboardChooser<Command> chooser) {
    return Commands.either(
        resetPose(
            resetPoseMap.getOrDefault(
                chooser.getSendableChooser().getSelected(), defaultTrajectory)),
        Commands.none(),
        () -> DriverStation.isDisabled() && ResetHeadingUtil.shouldReset());
  }

  public static void add(String name, AutoTrajectory traj) {
    resetPoseMap.put(name, traj);
  }

  public static void addDefault(AutoTrajectory traj) {
    defaultTrajectory = traj;
  }

  public static boolean shouldReset() {
    return resetHeading;
  }

  public static void finishedAuto() {
    resetHeading = false;
  }

  public static void resetPosePeriodic(LoggedDashboardChooser<Command> chooser) {
    timer.start();
    if (timer.hasElapsed(0.2)) {
      resetPose(chooser).schedule();
      timer.reset();
      return;
    }

    if (!resetHeading) {
      timer.reset();
    }
    chooser
        .getSendableChooser()
        .onChange(
            (key) -> {
              resetPose(chooser).schedule();
              timer.reset();
            });
  }
}
