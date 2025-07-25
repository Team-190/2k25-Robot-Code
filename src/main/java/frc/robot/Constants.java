package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final boolean TUNING_MODE = false;
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final RobotType ROBOT = RobotType.V2_REDUNDANCY_SIM;

  public static Mode getMode() {
    switch (ROBOT) {
      case V0_FUNKY:
      case V0_GOMPEIVISION_TEST:
      case V0_WHIPLASH:
      case V1_STACKUP:
      case V2_REDUNDANCY:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case V0_FUNKY_SIM:
      case V0_GOMPEIVISION_TEST_SIM:
      case V0_WHIPLASH_SIM:
      case V1_STACKUP_SIM:
      case V2_REDUNDANCY_SIM:
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
    V0_GOMPEIVISION_TEST,
    V0_GOMPEIVISION_TEST_SIM,
    V0_WHIPLASH,
    V0_WHIPLASH_SIM,
    V1_STACKUP,
    V1_STACKUP_SIM,
    V2_REDUNDANCY,
    V2_REDUNDANCY_SIM
  }

  public static void main(String... args) {
    if (ROBOT == RobotType.V1_STACKUP_SIM) {
      System.err.println("Cannot deploy, invalid mode selected: " + ROBOT.toString());
      System.exit(1);
    }
  }
}
