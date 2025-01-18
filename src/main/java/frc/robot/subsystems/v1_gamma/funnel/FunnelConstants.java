package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class FunnelConstants {
  public static final int CRAB_MOTOR_ID;
  public static final int INTAKE_MOTOR_ID;
  public static final int CORAL_SENSOR_ID;
  public static final Gains CRAB_MOTOR_GAINS;
  public static final Gains INTAKE_MOTOR_GAINS;
  public static final FunnelCurrentLimits CURRENT_LIMITS;
  public static final double CRAB_MOTOR_GEAR_RATIO;
  public static final double INTAKE_MOTOR_GEAR_RATIO;
  public static final Thresholds ANGLE_THRESHOLDS;
  public static final Constraints CRAB_MOTOR_CONSTRAINTS;
  public static final Constraints INTAKE_MOTOR_CONSTRAINTS;
  public static final LoggedTunableNumber CANCODER_ABSOLUTE_OFFSET_ROTATIONS;
  public static final FunnelParams CRAB_PARAMS = new FunnelParams(DCMotor.getKrakenX60(1), 0.004);
  public static final FunnelParams INTAKE_PARAMS = new FunnelParams(DCMotor.getKrakenX60(1), 0.004);

  static {
    CRAB_MOTOR_ID = 0;
    INTAKE_MOTOR_ID = 0;
    CORAL_SENSOR_ID = 0;
    CRAB_MOTOR_GAINS =
        new Gains(
            new LoggedTunableNumber("Funnel/Crab Motor Gains/kP", 0.0),
            new LoggedTunableNumber("Funnel/Crab Motor Gains/kD", 0.0),
            new LoggedTunableNumber("Funnel/Crab Motor Gains/kS", 0.0),
            new LoggedTunableNumber("Funnel/Crab Motor Gains/kV", 0.0),
            new LoggedTunableNumber("Funnel/Crab Motor Gains/kA", 0.0));
    INTAKE_MOTOR_GAINS =
        new Gains(
            new LoggedTunableNumber("Funnel/Intake Motor Gains/kP", 0.0),
            new LoggedTunableNumber("Funnel/Intake Motor Gains/kD", 0.0),
            new LoggedTunableNumber("Funnel/Intake Motor Gains/kS", 0.0),
            new LoggedTunableNumber("Funnel/Intake Motor Gains/kV", 0.0),
            new LoggedTunableNumber("Funnel/Intake Motor Gains/kA", 0.0));

    CURRENT_LIMITS = new FunnelCurrentLimits(0.0, 0.0, 0.0, 0.0);

    CRAB_MOTOR_GEAR_RATIO = 0.0;
    INTAKE_MOTOR_GEAR_RATIO = 0.0;

    ANGLE_THRESHOLDS =
        new Thresholds(
            new LoggedTunableNumber("Funnel/Crab Motor/Max Angle", 0.0),
            new LoggedTunableNumber("Funnel/Crab Motor/Min Angle", 0.0));
    CRAB_MOTOR_CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Funnel/Crab Motor/Max Acceleration", 0.0),
            new LoggedTunableNumber("Funnel/Crab Motor/Max Velocity", 0.0),
            new LoggedTunableNumber("Funnel/Goal Tolerance", 0.0));
    INTAKE_MOTOR_CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Funnel/Intake Motor/Max Acceleration", 0.0),
            new LoggedTunableNumber("Funnel/Intake Motor/Max Velocity", 0.0),
            new LoggedTunableNumber("Funnel/Goal Tolerance", 0.0));

    CANCODER_ABSOLUTE_OFFSET_ROTATIONS =
        new LoggedTunableNumber("Funnel/CanCoder Absolute Offset", 0.0);
  }

  public static final record FunnelCurrentLimits(
      double CRAB_SUPPLY_CURRENT_LIMIT,
      double INTAKE_SUPPLY_CURRENT_LIMIT,
      double CRAB_STATOR_CURRENT_LIMIT,
      double INTAKE_STATOR_CURRENT_LIMIT) {}

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
