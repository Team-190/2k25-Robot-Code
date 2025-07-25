package frc.robot.subsystems.v2_Redundancy.superstructure.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class V2_RedundancyIntakeConstants {
  public static final int EXTENSION_MOTOR_ID;
  public static final int ROLLER_MOTOR_ID;
  public static final double EXTENSION_MOTOR_GEAR_RATIO;
  public static final double EXTENSION_MOTOR_METERS_PER_REV;
  public static final double ROLLER_MOTOR_GEAR_RATIO;

  public static final IntakeCurrentLimits CURRENT_LIMITS;
  public static final Gains EXTENSION_MOTOR_GAINS;
  public static final Thresholds ANGLE_THRESHOLDS;
  public static final Constraints EXTENSION_MOTOR_CONSTRAINTS;
  public static final ExtensionParams EXTENSION_PARAMS;
  public static final RollerParams ROLLER_PARAMS;

  static {
    EXTENSION_MOTOR_ID = 60;
    ROLLER_MOTOR_ID = 61;
    EXTENSION_MOTOR_GEAR_RATIO = 3;
    EXTENSION_MOTOR_METERS_PER_REV = .0266 * EXTENSION_MOTOR_GEAR_RATIO;
    ROLLER_MOTOR_GEAR_RATIO = 2.5;

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
      default:
        CURRENT_LIMITS = new IntakeCurrentLimits(40.0, 40.0, 40.0, 40.0);
        EXTENSION_MOTOR_GAINS =
            new Gains(
                new LoggedTunableNumber("Intake/Extension Motor Gains/kP", 100.0),
                new LoggedTunableNumber("Intake/Extension Motor Gains/kD", 0.0),
                new LoggedTunableNumber("Intake/Extension Motor Gains/kS", 0.5),
                new LoggedTunableNumber("Intake/Extension Motor Gains/kV", 0.0),
                new LoggedTunableNumber("Intake/Extension Motor Gains/kA", 0.0));
        ANGLE_THRESHOLDS = new Thresholds(4.4, 0.0);
        EXTENSION_MOTOR_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Intake/Extension Motor/Max Acceleration", 500.0),
                new LoggedTunableNumber("Intake/Extension Motor/Max Velocity", 500.0),
                new LoggedTunableNumber("Intake/Goal Tolerance", 0.01));
        EXTENSION_PARAMS =
            new ExtensionParams(
                DCMotor.getKrakenX60(1),
                0.0042,
                Units.inchesToMeters(1.0),
                IntakeExtensionState.STOW.getDistance(),
                IntakeExtensionState.INTAKE.getDistance());
        ROLLER_PARAMS = new RollerParams(DCMotor.getKrakenX60(1), 0.0042);
        break;

      case SIM:
        CURRENT_LIMITS = new IntakeCurrentLimits(40.0, 40.0, 40.0, 40.0);
        EXTENSION_MOTOR_GAINS =
            new Gains(
                new LoggedTunableNumber("Intake/Extension Motor Gains/kP", 40),
                new LoggedTunableNumber("Intake/Extension Motor Gains/kD", 0.0),
                new LoggedTunableNumber("Intake/Extension Motor Gains/kS", 0.0),
                new LoggedTunableNumber("Intake/Extension Motor Gains/kV", 0.0),
                new LoggedTunableNumber("Intake/Extension Motor Gains/kA", 0.0));
        ANGLE_THRESHOLDS = new Thresholds(Units.degreesToRadians(90.0), 0.0);
        EXTENSION_MOTOR_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Intake/Extension Motor/Max Acceleration", 100.0),
                new LoggedTunableNumber("Intake/Extension Motor/Max Velocity", 100.0),
                new LoggedTunableNumber("Intake/Goal Tolerance", 0.0));
        EXTENSION_PARAMS =
            new ExtensionParams(
                DCMotor.getKrakenX60(1),
                0.0042,
                Units.inchesToMeters(1.0),
                IntakeExtensionState.STOW.getDistance(),
                IntakeExtensionState.INTAKE.getDistance());
        ROLLER_PARAMS = new RollerParams(DCMotor.getKrakenX60(1), 0.0042);
        break;
    }
  }

  public static final record IntakeCurrentLimits(
      double EXTENSION_SUPPLY_CURRENT_LIMIT,
      double ROLLER_SUPPLY_CURRENT_LIMIT,
      double EXTENSION_STATOR_CURRENT_LIMIT,
      double ROLLER_STATOR_CURRENT_LIMIT) {}

  public static final record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static final record Thresholds(
      double MAX_EXTENSION_ROTATIONS, double MIN_EXTENSION_ROTATIONS) {}

  public static final record Constraints(
      LoggedTunableNumber MAX_ACCELERATION,
      LoggedTunableNumber MAX_VELOCITY,
      LoggedTunableNumber GOAL_TOLERANCE) {}

  public static final record ExtensionParams(
      DCMotor motor,
      double massKg,
      double pitchDiameter,
      double minExtension,
      double maxExtension) {}

  public static final record RollerParams(DCMotor motor, double momentOfInertia) {}

  @RequiredArgsConstructor
  public enum IntakeExtensionState {
    STOW(0),
    INTAKE(0.3496120605),
    L1_EXT(0.0689 - Units.inchesToMeters(.5));

    private final double distanceMeters;

    public double getDistance() {
      return distanceMeters;
    }
  }

  @RequiredArgsConstructor
  public enum IntakeRollerState {
    STOP(0.0),
    INTAKE(6.0),
    RETRACT(-2.0),
    OUTTAKE(-6.0);

    private final double voltage;

    public double getVoltage() {
      return voltage;
    }
  }
}
