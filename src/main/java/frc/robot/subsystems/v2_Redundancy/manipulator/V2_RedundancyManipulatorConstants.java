package frc.robot.subsystems.v2_Redundancy.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.LoggedTunableNumber;

public class V2_RedundancyManipulatorConstants {
  public static final int ARM_CAN_ID;
  public static final double ARM_SUPPLY_CURRENT_LIMIT;
  public static final ArmParameters ARM_PARAMETERS;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  public static final int ROLLER_CAN_ID;
  public static final double ROLLER_SUPPLY_CURRENT_LIMIT;
  public static final double ROLLER_CURRENT_THRESHOLD;
  public static final Rotation2d ROLLER_TOGGLE_ARM_ROTATION;
  public static final Voltages ROLLER_VOLTAGES;

  public static final ManipulatorCurrentLimits CURRENT_LIMITS;

  static {
    ARM_CAN_ID = 31;
    ARM_SUPPLY_CURRENT_LIMIT = 40;
    ARM_PARAMETERS =
        new ArmParameters(
            DCMotor.getKrakenX60Foc(1), Rotation2d.fromDegrees(-77.0), Rotation2d.fromDegrees(75.0), 1, 90.0, 0.5);
    GAINS =
        new Gains(
            new LoggedTunableNumber("Manipulator/Arm/kP", 0.0),
            new LoggedTunableNumber("Manipulator/Arm/kD", 0.0),
            new LoggedTunableNumber("Manipulator/Arm/kS", 0.0),
            new LoggedTunableNumber("Manipulator/Arm/kG", 0.0),
            new LoggedTunableNumber("Manipulator/Arm/kV", 0.0),
            new LoggedTunableNumber("Manipulator/Arm/kA", 0.0));
    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Manipulator/Arm/MaxAcceleration", 0.0),
            new LoggedTunableNumber("Manipulator/Arm/CruisingVelocity", 0.0),
            new LoggedTunableNumber("Manipulator/Arm/GoalTolerance", 0.0));

    ROLLER_CAN_ID = 30;
    ROLLER_SUPPLY_CURRENT_LIMIT = 20.0;
    ROLLER_CURRENT_THRESHOLD = 60.0;
    ROLLER_TOGGLE_ARM_ROTATION = Rotation2d.fromRadians(10);
    ROLLER_VOLTAGES =
        new Voltages(
            new LoggedTunableNumber("Manipulator/Intake Volts", 6.0),
            new LoggedTunableNumber("Manipulator/Score Volts", 4.0),
            new LoggedTunableNumber("Manipulator/Remove Algae Volts", 12),
            new LoggedTunableNumber("Manipulator/HalfScore Volts", 1.0),
            new LoggedTunableNumber("Manipulator/L1 Volts", 3.5));

    CURRENT_LIMITS = new ManipulatorCurrentLimits(40, 40, 20, 20);
  }

  public static record ArmParameters(
      DCMotor MOTOR_CONFIG,
      Rotation2d MIN_ANGLE,
      Rotation2d MAX_ANGLE,
      int NUM_MOTORS,
      double GEAR_RATIO,
      double LENGTH_METERS) {}

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static final record ManipulatorCurrentLimits(
      double MANIPULATOR_SUPPLY_CURRENT_LIMIT,
      double ROLLER_SUPPLY_CURRENT_LIMIT,
      double MANIPULATOR_STATOR_CURRENT_LIMIT,
      double ROLLER_STATOR_CURRENT_LIMIT) {}

  public static record Constraints(
      LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      LoggedTunableNumber cruisingVelocityRadiansPerSecond,
      LoggedTunableNumber goalToleranceRadians) {}

  public static final record Voltages(
      LoggedTunableNumber INTAKE_VOLTS,
      LoggedTunableNumber SCORE_VOLTS,
      LoggedTunableNumber REMOVE_ALGAE,
      LoggedTunableNumber HALF_VOLTS,
      LoggedTunableNumber L1_VOLTS) {}
}
