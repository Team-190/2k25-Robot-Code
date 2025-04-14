package frc.robot.subsystems.shared.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberConstants {
  public static final int MOTOR_ID;

  public static final DCMotor MOTOR_CONFIG;

  public static final double CLIMBER_SUPPLY_CURRENT_LIMIT;
  public static final double CLIMBER_STATOR_CURRENT_LIMIT;

  public static final double CLIMBER_CLIMBED_RADIANS;

  public static final double GEAR_RATIO;
  public static final double GEARBOX_EFFICIENCY;

  public static final double SPOOL_DIAMETER;

  public static final double WAIT_AFTER_RELEASE_SECONDS;
  public static final double REDUNDANCY_DELAY_SECONDS;
  public static final double REDUNDANCY_TRUSTING_TIMEOUT_SECONDS;

  static {
    switch (Constants.ROBOT) {
      case V1_STACKUP:
      case V1_STACKUP_SIM:
        MOTOR_ID = 50;

        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);

        CLIMBER_SUPPLY_CURRENT_LIMIT = 40.0;
        CLIMBER_STATOR_CURRENT_LIMIT = 80.0;

        CLIMBER_CLIMBED_RADIANS = 415;

        GEAR_RATIO = 24.0;
        GEARBOX_EFFICIENCY = 0.81;

        SPOOL_DIAMETER = Units.inchesToMeters(1.78);

        WAIT_AFTER_RELEASE_SECONDS = 1.1;
        REDUNDANCY_DELAY_SECONDS = 0.25;
        REDUNDANCY_TRUSTING_TIMEOUT_SECONDS = 0.5;
        break;
      case V2_REDUNDANCY:
      case V2_REDUNDANCY_SIM:
      default:
        MOTOR_ID = 50;

        MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);

        CLIMBER_SUPPLY_CURRENT_LIMIT = 40.0;
        CLIMBER_STATOR_CURRENT_LIMIT = 80.0;

        CLIMBER_CLIMBED_RADIANS = 410;

        GEAR_RATIO = 24.0;
        GEARBOX_EFFICIENCY = 0.81;

        SPOOL_DIAMETER = Units.inchesToMeters(1.78);

        WAIT_AFTER_RELEASE_SECONDS = 1.1;
        REDUNDANCY_DELAY_SECONDS = 0.25;
        REDUNDANCY_TRUSTING_TIMEOUT_SECONDS = 0.5;
        break;
    }
  }
}
