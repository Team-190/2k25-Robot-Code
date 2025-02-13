package frc.robot.subsystems.v1_gamma.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class V1_GammaClimberConstants {
  public static final int MOTOR_ID;

  public static final DCMotor MOTOR_CONFIG;

  public static final double CLIMBER_SUPPLY_CURRENT_LIMIT;
  public static final double CLIMBER_STATOR_CURRENT_LIMIT;

  public static final double CLIMBER_CLIMBED_RADIANS;

  public static final double GEAR_RATIO;
  public static final double GEARBOX_EFFICIENCY;

  public static final double SPOOL_DIAMETER;

  static {
    MOTOR_ID = 50;

    MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);

    CLIMBER_SUPPLY_CURRENT_LIMIT = 40.0;
    CLIMBER_STATOR_CURRENT_LIMIT = 40.0;

    CLIMBER_CLIMBED_RADIANS = 42.0;

    GEAR_RATIO = 24.0;
    GEARBOX_EFFICIENCY = 0.81;

    SPOOL_DIAMETER = Units.inchesToMeters(1.78);
  }
}
