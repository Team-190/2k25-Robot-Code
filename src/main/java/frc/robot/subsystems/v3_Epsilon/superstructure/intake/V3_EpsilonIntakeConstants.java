package frc.robot.subsystems.v3_Epsilon.superstructure.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class V3_EpsilonIntakeConstants {

  public static final int PIVOT_CAN_ID;
  public static final int ROLLER_CAN_ID_OUTER;
  public static final int ROLLER_CAN_ID_INNER;
  public static final int LEFT_SENSOR_CAN_ID;
  public static final int RIGHT_SENSOR_CAN_ID;

  public static final double INTAKE_CAN_CORAL_DETECTED_THRESHOLD_METERS;

  public static final IntakeCurrentLimits CURRENT_LIMITS;

  public static final Gains PIVOT_GAINS;
  public static final Constraints PIVOT_CONSTRAINTS;

  public static final IntakeParems PIVOT_PARAMS;
  public static final IntakeParems ROLLER_PARAMS;

  static {
    PIVOT_CAN_ID = 60;
    ROLLER_CAN_ID_OUTER = 61; // TODO: Check numbers here
    ROLLER_CAN_ID_INNER = 62; // TODO: Check numbers here
    LEFT_SENSOR_CAN_ID = 0; // TODO: Check numbers here
    RIGHT_SENSOR_CAN_ID = 1;

    INTAKE_CAN_CORAL_DETECTED_THRESHOLD_METERS = 0.05; // TODO: Check this value

    PIVOT_PARAMS =
        new IntakeParems(
            3.0,
            DCMotor.getKrakenX60Foc(1),
            0.0042,
            Rotation2d.fromDegrees(0.0),
            Rotation2d.fromDegrees(124.6));
    ROLLER_PARAMS =
        new IntakeParems(
            1, DCMotor.getKrakenX60Foc(1), 0, new Rotation2d(), Rotation2d.fromDegrees(0));

    switch (Constants.ROBOT) {
      case V3_EPSILON_SIM:
        PIVOT_CONSTRAINTS = new Constraints(500.0, 500.0, Rotation2d.fromDegrees(1.5));
        PIVOT_GAINS = new Gains(0.0, 0.00, 0.0, 0.0, 0.0, 0.0);
        CURRENT_LIMITS = new IntakeCurrentLimits(40.0, 40.0, 40.0, 40.0, 40.0, 40.0);

        break;

      default:
        PIVOT_CONSTRAINTS = new Constraints(500.0, 500.0, Rotation2d.fromDegrees(1.5));
        PIVOT_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        CURRENT_LIMITS = new IntakeCurrentLimits(40.0, 40.0, 40.0, 40.0, 40.0, 40.0);

        break;
    }
  }

  @RequiredArgsConstructor
  public enum IntakePivotState {
    STOW(Rotation2d.fromDegrees(-123.6)),
    INTAKE_CORAL(Rotation2d.fromDegrees(0)),
    HANDOFF(Rotation2d.fromDegrees(-123.6)),
    L1(Rotation2d.fromDegrees(-82)),
    INTAKE_ALGAE(new Rotation2d(0)),
    ARM_CLEAR(Rotation2d.fromDegrees(35));

    @Getter private final Rotation2d angle;
  }

  public static record IntakeCurrentLimits(
      double PIVOT_SUPPLY_CURRENT_LIMIT,
      double PIVOT_STATOR_CURRENT_LIMIT,
      double INNER_ROLLER_SUPPLY_CURRENT_LIMIT,
      double INNER_ROLLER_STATOR_CURRENT_LIMIT,
      double OUTER_ROLLER_SUPPLY_CURRENT_LIMIT,
      double OUTER_ROLLER_STATOR_CURRENT_LIMIT) {}

  public static record Gains(double kP, double kD, double kS, double kV, double kA, double kG) {}

  public static record Constraints(
      double MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED,
      double CRUISING_VELOCITY_RADIANS_PER_SECOND,
      Rotation2d GOAL_TOLERANCE) {
    public edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints getTrapezoidConstraints() {
      return new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
          CRUISING_VELOCITY_RADIANS_PER_SECOND, MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }
  }

  public static record IntakeParems(
      double PIVOT_GEAR_RATIO,
      DCMotor MOTOR,
      double MASS_KG,
      Rotation2d MIN_ANGLE,
      Rotation2d MAX_ANGLE) {}

  // Will add more states later
  public static enum IntakeRollerState {
    STOP(0.0, 0.0),
    CORAL_INTAKE(6.0, 6.0),
    ALGAE_INTAKE(12.0, 12.0),
    SCORE_CORAL(6.0, 6.0),
    OUTTAKE(10.0, 10.0);

    @Getter private final double innerVoltage;
    @Getter private final double outerVoltage;

    IntakeRollerState(double innerVoltage, double outerVoltage) {
      this.innerVoltage = innerVoltage;
      this.outerVoltage = outerVoltage;
    }
  }
}
