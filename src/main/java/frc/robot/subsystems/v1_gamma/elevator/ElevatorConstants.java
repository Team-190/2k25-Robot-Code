package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class ElevatorConstants {
  public static final int ELEVATOR_CAN_ID;
  public static final double ELEVATOR_GEAR_RATIO;
  public static final double DRUM_RADIUS;

  public static final ElevatorParams ELEVATOR_PARAMS;

  public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT;
  public static final double ELEVATOR_STATOR_CURRENT_LIMIT;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  static {
    ELEVATOR_CAN_ID = 0;
    ELEVATOR_GEAR_RATIO = 1.0;
    DRUM_RADIUS = Units.inchesToMeters(2.256);

    ELEVATOR_PARAMS =
        new ElevatorParams(
            DCMotor.getKrakenX60Foc(4), 1, 0, Units.inchesToMeters(58), 4); // Will change

    ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;
    ELEVATOR_STATOR_CURRENT_LIMIT = 40;

    GAINS =
        new Gains(
            new LoggedTunableNumber("Elevator/Gains/kP", 0.1),
            new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
            new LoggedTunableNumber("Elevator/Gains/kS", 0.0),
            new LoggedTunableNumber("Elevator/Gains/kG", 0.0),
            new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
            new LoggedTunableNumber("Elevator/Gains/kA", 0.0));

    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Elevator/Max Acceleration", 4),
            new LoggedTunableNumber("Elevator/Cruising Velocity", 1),
            new LoggedTunableNumber("Elevator/Goal Tolerance", 0.01));
  }

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static record Constraints(
      LoggedTunableNumber maxAccelerationRotsPerSecSq,
      LoggedTunableNumber cruisingVelocityRotsPerSec,
      LoggedTunableNumber goalToleranceMeters) {}

  public static record ElevatorParams(
      DCMotor ELEVATOR_MOTOR_CONFIG,
      double CARRIAGE_MASS_KG,
      double MIN_HEIGHT_METERS,
      double MAX_HEIGHT_METERS,
      int NUM_MOTORS) {}

  @RequiredArgsConstructor
  public static enum ElevatorPositions {
    STOW(0.0),
    INTAKE(15.0),
    L1(10.0),
    L2(25.0),
    L3(50.0),
    L4(75.0);

    private final double position;

    public double getPosition() {
      return position;
    }
  }
}
