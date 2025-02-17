package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class V1_GammaFunnelConstants {
  public static final int CLAP_DADDY_MOTOR_ID;
  public static final int ROLLER_MOTOR_ID;
  public static final int CORAL_SENSOR_ID;
  public static final int CLAP_DADDY_CANCODER_ID;
  public static final double CLAP_DADDY_MOTOR_GEAR_RATIO;
  public static final double ROLLER_MOTOR_GEAR_RATIO;
  public static final double CLAP_DADDY_CANCODER_GEAR_RATIO;
  public static final Rotation2d CANCODER_ABSOLUTE_OFFSET_RADIANS;

  public static final FunnelCurrentLimits CURRENT_LIMITS;
  public static final Gains CLAP_DADDY_MOTOR_GAINS;
  public static final Thresholds ANGLE_THRESHOLDS;
  public static final Constraints CLAP_DADDY_MOTOR_CONSTRAINTS;
  public static final FunnelParams CLAP_DADDY_PARAMS;
  public static final FunnelParams ROLLER_PARAMS;

  public static final double ROLLER_VOLTS;
  public static final double ROLLER_OFFSET_INCREMENT_VOLTS;
  public static final double CLAP_DADDY_OFFSET_INCREMENT_RADIANS;

  static {
    CLAP_DADDY_MOTOR_ID = 41;
    ROLLER_MOTOR_ID = 40;
    CORAL_SENSOR_ID = 0;
    CLAP_DADDY_CANCODER_ID = 40;
    CLAP_DADDY_MOTOR_GEAR_RATIO = 17.0;
    ROLLER_MOTOR_GEAR_RATIO = 2.5;
    CLAP_DADDY_CANCODER_GEAR_RATIO = 3.0;
    CANCODER_ABSOLUTE_OFFSET_RADIANS = Rotation2d.fromRadians(4.95);

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
      default:
        CURRENT_LIMITS = new FunnelCurrentLimits(40.0, 40.0, 40.0, 40.0);
        CLAP_DADDY_MOTOR_GAINS =
            new Gains(
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kP", 125.0),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kD", 5.0),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kS", 0.224),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kV", 0.0),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kA", 0.0));
        ANGLE_THRESHOLDS = new Thresholds(Units.degreesToRadians(90.0), 0.0);
        CLAP_DADDY_MOTOR_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Funnel/Clap Daddy Motor/Max Acceleration", 100.0),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor/Max Velocity", 100.0),
                new LoggedTunableNumber("Funnel/Goal Tolerance", 0.0));
        CLAP_DADDY_PARAMS = new FunnelParams(DCMotor.getKrakenX60(1), 0.0042);
        ROLLER_PARAMS = new FunnelParams(DCMotor.getKrakenX60(1), 0.0042);
        ROLLER_VOLTS = 12.0;
        ROLLER_OFFSET_INCREMENT_VOLTS = 0.1;
        CLAP_DADDY_OFFSET_INCREMENT_RADIANS = Units.degreesToRadians(1);
        break;

      case SIM:
        CURRENT_LIMITS = new FunnelCurrentLimits(40.0, 40.0, 40.0, 40.0);
        CLAP_DADDY_MOTOR_GAINS =
            new Gains(
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kP", 40),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kD", 0.0),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kS", 0.0),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kV", 0.0),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor Gains/kA", 0.0));
        ANGLE_THRESHOLDS = new Thresholds(Units.degreesToRadians(90.0), 0.0);
        CLAP_DADDY_MOTOR_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Funnel/Clap Daddy Motor/Max Acceleration", 100.0),
                new LoggedTunableNumber("Funnel/Clap Daddy Motor/Max Velocity", 100.0),
                new LoggedTunableNumber("Funnel/Goal Tolerance", 0.0));
        CLAP_DADDY_PARAMS = new FunnelParams(DCMotor.getKrakenX60(1), 0.0042);
        ROLLER_PARAMS = new FunnelParams(DCMotor.getKrakenX60(1), 0.0042);
        ROLLER_VOLTS = 12.0;
        ROLLER_OFFSET_INCREMENT_VOLTS = 0.1;
        CLAP_DADDY_OFFSET_INCREMENT_RADIANS = Units.degreesToRadians(1);
        break;
    }
  }

  public static final record FunnelCurrentLimits(
      double CLAP_DADDY_SUPPLY_CURRENT_LIMIT,
      double ROLLER_SUPPLY_CURRENT_LIMIT,
      double CLAP_DADDY_STATOR_CURRENT_LIMIT,
      double ROLLER_STATOR_CURRENT_LIMIT) {}

  public static final record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static final record Thresholds(double MAX_ANGLE_RADIANS, double MIN_ANGLE_RADIANS) {}

  public static final record Constraints(
      LoggedTunableNumber MAX_ACCELERATION,
      LoggedTunableNumber MAX_VELOCITY,
      LoggedTunableNumber GOAL_TOLERANCE) {}

  public static final record FunnelParams(DCMotor motor, double momentOfInertia) {}

  @RequiredArgsConstructor
  public enum FunnelState {
    STOW(Rotation2d.fromDegrees(65.0)),
    OPENED(Rotation2d.fromDegrees(65.0)),
    CLOSED(Rotation2d.fromDegrees(90.0)),
    CLIMB(Rotation2d.fromDegrees(0.0));

    private final Rotation2d angle;
    private double offset;

    public Rotation2d getAngle() {
      return Rotation2d.fromRadians(angle.getRadians() + offset);
    }

    public void setOffset(double offset) {
      this.offset += offset;
    }
  }
}
