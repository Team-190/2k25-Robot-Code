package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;

public class ResetHeadingUtil {
  private static boolean resetHeading;
  private static final Timer timer;

  static {
    resetHeading = true;
    timer = new Timer();
  }


  public static void resetPose() {
    if (DriverStation.isDisabled() && ResetHeadingUtil.shouldReset())
        RobotState.resetRobotPose(RobotState.getMT1RobotPoseField());
  }

  public static boolean shouldReset() {
    return resetHeading;
  }

  public static void finishedAuto() {
    resetHeading = false;
  }

  public static void resetPosePeriodic() {
    timer.start();
    if (timer.hasElapsed(0.2)) {
      resetPose();
      timer.reset();
      return;
    }

    if (!resetHeading) {
      timer.reset();
    }
  }
}
