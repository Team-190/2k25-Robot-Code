package frc.robot.subsystems.v3_Epsilon.superstructure.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
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
    PIVOT_CAN_ID = 40;
    ROLLER_CAN_ID_OUTER = 41;
    ROLLER_CAN_ID_INNER = 42;
    LEFT_SENSOR_CAN_ID = 44;
    RIGHT_SENSOR_CAN_ID = 43;

    INTAKE_CAN_CORAL_DETECTED_THRESHOLD_METERS = 0.05;

    PIVOT_PARAMS =
        new IntakeParems(
            (60.0 / 12.0) * (52.0 / 28.0) * (64.0 / 18.0) * (20.0 / 9.0),
            DCMotor.getKrakenX60Foc(1),
            0.0042,
            Rotation2d.fromDegrees(40.0),
            Rotation2d.fromDegrees(124.6 + 48));
    ROLLER_PARAMS =
        new IntakeParems(
            1, DCMotor.getKrakenX60Foc(1), 0, new Rotation2d(), Rotation2d.fromDegrees(0));

    switch (Constants.ROBOT) {
      case V3_EPSILON:
        PIVOT_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Intake/Max Acceleration", 160),
                new LoggedTunableNumber("Intake/Cruising Velocity", 80),
                Rotation2d.fromDegrees(1.5));
        PIVOT_GAINS =
            new Gains(
                new LoggedTunableNumber("Intake/kP", 175.0),
                new LoggedTunableNumber("Intake/kD", 5.0),
                new LoggedTunableNumber("Intake/kS", 0.38466),
                new LoggedTunableNumber("Intake/kV", 0.0),
                new LoggedTunableNumber("Intake/kA", 0),
                new LoggedTunableNumber("Intake/kG", 0.083386));
        CURRENT_LIMITS = new IntakeCurrentLimits(40.0, 40.0, 40.0, 40.0, 40.0, 40.0);

        break;

      case V3_EPSILON_SIM:
        PIVOT_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Intake/Max Acceleration", 100.0),
                new LoggedTunableNumber("Intake/Cruising Velocity", 75.0),
                Rotation2d.fromDegrees(1.5));
        PIVOT_GAINS =
            new Gains(
                new LoggedTunableNumber("Intake/kP", 1.85),
                new LoggedTunableNumber("Intake/kD", 0.1),
                new LoggedTunableNumber("Intake/kS", 0.0),
                new LoggedTunableNumber("Intake/kV", 0.0),
                new LoggedTunableNumber("Intake/kA", 0.011537),
                new LoggedTunableNumber("Intake/kG", 0.15326));
        CURRENT_LIMITS = new IntakeCurrentLimits(40.0, 40.0, 40.0, 40.0, 40.0, 40.0);

        break;

      default:
        PIVOT_CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Intake/Max Acceleration", 0.0),
                new LoggedTunableNumber("Intake/Cruising Velocity", 0.0),
                Rotation2d.fromDegrees(1.5));
        PIVOT_GAINS =
            new Gains(
                new LoggedTunableNumber("Intake/kP", 1.85),
                new LoggedTunableNumber("Intake/kD", 0.1),
                new LoggedTunableNumber("Intake/kS", 0.0),
                new LoggedTunableNumber("Intake/kV", 0.0),
                new LoggedTunableNumber("Intake/kA", 0.0),
                new LoggedTunableNumber("Intake/kG", 0.0));
        CURRENT_LIMITS = new IntakeCurrentLimits(40.0, 40.0, 40.0, 40.0, 40.0, 40.0);

        break;
    }
  }

  @RequiredArgsConstructor
  public enum IntakePivotState {
    STOW(Rotation2d.fromDegrees(-82 + 123.6 + 48)),
    INTAKE_CORAL(Rotation2d.fromDegrees(123.6 + 48)),
    HANDOFF(Rotation2d.fromDegrees(48)),
    L1(Rotation2d.fromDegrees(-82 + 123.6 + 48)),
    INTAKE_ALGAE(Rotation2d.fromDegrees(25.0 + 48)),
    ARM_CLEAR(Rotation2d.fromDegrees(35 + 48));

    @Getter private final Rotation2d angle;
  }

  public static record IntakeCurrentLimits(
      double PIVOT_SUPPLY_CURRENT_LIMIT,
      double PIVOT_STATOR_CURRENT_LIMIT,
      double INNER_ROLLER_SUPPLY_CURRENT_LIMIT,
      double INNER_ROLLER_STATOR_CURRENT_LIMIT,
      double OUTER_ROLLER_SUPPLY_CURRENT_LIMIT,
      double OUTER_ROLLER_STATOR_CURRENT_LIMIT) {}

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA,
      LoggedTunableNumber kG) {}

  public static record Constraints(
      LoggedTunableNumber MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED,
      LoggedTunableNumber CRUISING_VELOCITY_RADIANS_PER_SECOND,
      Rotation2d GOAL_TOLERANCE) {
    public edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints getTrapezoidConstraints() {
      return new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
          CRUISING_VELOCITY_RADIANS_PER_SECOND.get(),
          MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get());
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
    CORAL_INTAKE(-12.0, -12.0),
    ALGAE_INTAKE(12.0, 12.0),
    SCORE_CORAL(6.0, 6.0),
    OUTTAKE(12.0, 12.0);

    @Getter private final double innerVoltage;
    @Getter private final double outerVoltage;

    IntakeRollerState(double innerVoltage, double outerVoltage) {
      this.innerVoltage = innerVoltage;
      this.outerVoltage = outerVoltage;
    }
  }
}
