package frc.robot.subsystems.shared.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberConstants {
  public static final int MOTOR_ID;
  public static final MotorParameters MOTOR_PARAMETERS;
  
  public static final double CLIMBER_CLIMBED_RADIANS;
  public static final ClimberTimingConfig CLIMBER_TIMING_CONFIG;

  public static final CurrentLimits CURRENT_LIMITS;

  static {
    switch (Constants.ROBOT) {
      case V1_STACKUP:
      case V1_STACKUP_SIM:
        MOTOR_ID = 50;
        MOTOR_PARAMETERS = new MotorParameters(
            DCMotor.getKrakenX60Foc(1),
            24.0,
            0.81,
            Units.inchesToMeters(1.78)
        );
        
        CLIMBER_CLIMBED_RADIANS = 415;
        CLIMBER_TIMING_CONFIG = new ClimberTimingConfig(1.1, 0.25, 0.5);
        
        CURRENT_LIMITS = new CurrentLimits(40.0, 80.0);
        break;
        
      case V2_REDUNDANCY:
      case V2_REDUNDANCY_SIM:
      default:
        MOTOR_ID = 50;
        MOTOR_PARAMETERS = new MotorParameters(
            DCMotor.getKrakenX60Foc(1),
            24.0,
            0.81,
            Units.inchesToMeters(1.78)
        );
        
        CLIMBER_CLIMBED_RADIANS = 410;
        CLIMBER_TIMING_CONFIG = new ClimberTimingConfig(1.1, 0.25, 0.5);
        
        CURRENT_LIMITS = new CurrentLimits(40.0, 80.0);
        break;
    }
  }

  public static record MotorParameters(
      DCMotor MOTOR_CONFIG,
      double GEAR_RATIO,
      double GEARBOX_EFFICIENCY,
      double SPOOL_DIAMETER) {}

  public static record CurrentLimits(
      double SUPPLY_CURRENT_LIMIT,
      double STATOR_CURRENT_LIMIT) {}

  public static record ClimberTimingConfig(
      double WAIT_AFTER_RELEASE_SECONDS,
      double REDUNDANCY_DELAY_SECONDS,
      double REDUNDANCY_TRUSTING_TIMEOUT_SECONDS) {}
}
