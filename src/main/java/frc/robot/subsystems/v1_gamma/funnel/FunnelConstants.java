package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class FunnelConstants {
  public static final int SERIALIZER_MOTOR_ID;
  public static final int ROLLER_MOTOR_ID;
  public static final int CORAL_SENSOR_ID;
  public static final Gains SERIALIZER_MOTOR_GAINS;
  public static final FunnelCurrentLimits CURRENT_LIMITS;
  public static final double SERIALIZER_MOTOR_GEAR_RATIO;
  public static final double SERIALIZER_CANCODER_GEAR_RATIO;
  public static final double ROLLER_MOTOR_GEAR_RATIO;
  public static final Thresholds ANGLE_THRESHOLDS;
  public static final Constraints SERIALIZER_MOTOR_CONSTRAINTS;
  public static final LoggedTunableNumber CANCODER_ABSOLUTE_OFFSET_RADIANS;
  public static final FunnelParams SERIALIZER_PARAMS =
      new FunnelParams(DCMotor.getKrakenX60(1), 0.004);
  public static final FunnelParams ROLLER_PARAMS = new FunnelParams(DCMotor.getKrakenX60(1), 0.004);

  static {
    SERIALIZER_MOTOR_ID = 0;
    ROLLER_MOTOR_ID = 0;
    CORAL_SENSOR_ID = 0;
    SERIALIZER_MOTOR_GAINS =
        new Gains(
            new LoggedTunableNumber("Funnel/Serializer Motor Gains/kP", 0.0),
            new LoggedTunableNumber("Funnel/Serializer Motor Gains/kD", 0.0),
            new LoggedTunableNumber("Funnel/Serializer Motor Gains/kS", 0.0),
            new LoggedTunableNumber("Funnel/Serializer Motor Gains/kV", 0.0),
            new LoggedTunableNumber("Funnel/Serializer Motor Gains/kA", 0.0));

    CURRENT_LIMITS = new FunnelCurrentLimits(0.0, 0.0, 0.0, 0.0);

    SERIALIZER_MOTOR_GEAR_RATIO = 0.0;
    SERIALIZER_CANCODER_GEAR_RATIO = 0.0;
    ROLLER_MOTOR_GEAR_RATIO = 0.0;

    ANGLE_THRESHOLDS =
        new Thresholds(
            new LoggedTunableNumber("Funnel/Serializer Motor/Max Angle", 0.0),
            new LoggedTunableNumber("Funnel/Serializer Motor/Min Angle", 0.0));
    SERIALIZER_MOTOR_CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Funnel/Serializer Motor/Max Acceleration", 0.0),
            new LoggedTunableNumber("Funnel/Serializer Motor/Max Velocity", 0.0),
            new LoggedTunableNumber("Funnel/Goal Tolerance", 0.0));

    CANCODER_ABSOLUTE_OFFSET_RADIANS =
        new LoggedTunableNumber("Funnel/CanCoder Absolute Offset", 0.0);
  }

  public static final record FunnelCurrentLimits(
      double SERIALIZER_SUPPLY_CURRENT_LIMIT,
      double ROLLER_SUPPLY_CURRENT_LIMIT,
      double SERIALIZER_STATOR_CURRENT_LIMIT,
      double ROLLER_STATOR_CURRENT_LIMIT) {}

  public static final record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static final record Thresholds(
      LoggedTunableNumber MAX_ANGLE_RADIANS, LoggedTunableNumber MIN_ANGLE_RADIANS) {}

  public static final record Constraints(
      LoggedTunableNumber MAX_ACCELERATION,
      LoggedTunableNumber MAX_VELOCITY,
      LoggedTunableNumber GOAL_TOLERANCE) {}

  public static final record FunnelParams(DCMotor motor, double momentOfInertia) {}

  @RequiredArgsConstructor
  public enum FunnelState {
    STOW(35.0),
    OPENED(Units.degreesToRadians(15.0)),
    CLOSED(Units.degreesToRadians(85.0)),
    CLIMB(Units.degreesToRadians(0.0));

    private final double angleRadians;

    public double getAngle() {
      return angleRadians;
    }
  }
}
