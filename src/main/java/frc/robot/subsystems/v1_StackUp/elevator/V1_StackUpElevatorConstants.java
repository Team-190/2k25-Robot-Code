package frc.robot.subsystems.v1_StackUp.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class V1_StackUpElevatorConstants {
  public static final int ELEVATOR_CAN_ID;
  public static final double ELEVATOR_GEAR_RATIO;
  public static final double DRUM_RADIUS;

  public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT;
  public static final double ELEVATOR_STATOR_CURRENT_LIMIT;

  public static final ElevatorParameters ELEVATOR_PARAMETERS;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  static {
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
      default:
        ELEVATOR_CAN_ID = 20;
        ELEVATOR_GEAR_RATIO = 3.0;
        DRUM_RADIUS = Units.inchesToMeters(2.256 / 2.0);

        ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;
        ELEVATOR_STATOR_CURRENT_LIMIT = 40;

        ELEVATOR_PARAMETERS =
            new ElevatorParameters(DCMotor.getKrakenX60Foc(4), 6.803886, 0.0, 1.43, 4);
        GAINS =
            new Gains(
                new LoggedTunableNumber("Elevator/Gains/kP", 2.0),
                new LoggedTunableNumber("Elevator/Gains/kD", 0.1),
                new LoggedTunableNumber("Elevator/Gains/kS", 0.225),
                new LoggedTunableNumber("Elevator/Gains/kG", 0.075),
                new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
        CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Elevator/Max Acceleration", 16.0),
                new LoggedTunableNumber("Elevator/Cruising Velocity", 16.0),
                new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
        break;
      case SIM:
        ELEVATOR_CAN_ID = 20;
        ELEVATOR_GEAR_RATIO = 3.0;
        DRUM_RADIUS = Units.inchesToMeters(2.256);

        ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;
        ELEVATOR_STATOR_CURRENT_LIMIT = 40;

        ELEVATOR_PARAMETERS =
            new ElevatorParameters(
                DCMotor.getKrakenX60Foc(4), 6.803886, 0, Units.inchesToMeters(61.5), 4);
        GAINS =
            new Gains(
                new LoggedTunableNumber("Elevator/Gains/kP", 20.0),
                new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kS", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kG", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
        CONSTRAINTS =
            new Constraints(
                new LoggedTunableNumber("Elevator/Max Acceleration", 101.078594),
                new LoggedTunableNumber("Elevator/Cruising Velocity", 11.329982),
                new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
    }
  }

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static record Constraints(
      LoggedTunableNumber maxAccelerationMetersPerSecondSquared,
      LoggedTunableNumber cruisingVelocityMetersPerSecond,
      LoggedTunableNumber goalToleranceMeters) {}

  public static record ElevatorParameters(
      DCMotor ELEVATOR_MOTOR_CONFIG,
      double CARRIAGE_MASS_KG,
      double MIN_HEIGHT_METERS,
      double MAX_HEIGHT_METERS,
      int NUM_MOTORS) {}

  @RequiredArgsConstructor
  public static enum ElevatorPositions {
    STOW(0.0),
    INTAKE(0.0),
    TOP_ALGAE(1.2),
    BOT_ALGAE(0.82),
    L1(0.11295250319916351),
    L2(0.37296301250898894),
    L3(0.7606347556550676 + Units.inchesToMeters(1.0)),
    L4(1.3864590139769697 + Units.inchesToMeters(0.5));

    private final double position;

    public double getPosition() {
      return position;
    }
  }
}
