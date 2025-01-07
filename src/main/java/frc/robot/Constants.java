package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final boolean TUNING_MODE = false;
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final RobotType ROBOT = RobotType.V1_GAMMA;

  public static Mode getMode() {
    switch (ROBOT) {
      case V1_GAMMA:
      case V2_DELTA:
      case V0_FUNKY:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case V1_GAMMA_SIM:
      case V2_DELTA_SIM:
      case V0_FUNKY_SIM:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotType {
    V0_FUNKY,
    V0_FUNKY_SIM,
    V1_GAMMA,
    V1_GAMMA_SIM,
    V2_DELTA,
    V2_DELTA_SIM
  }

  public static void main(String... args) {
    if (ROBOT == RobotType.V1_GAMMA_SIM) {
      System.err.println("Cannot deploy, invalid mode selected: " + ROBOT.toString());
      System.exit(1);
    }
  }
}
