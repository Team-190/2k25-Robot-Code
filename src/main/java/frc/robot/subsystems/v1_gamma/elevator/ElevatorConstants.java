package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class ElevatorConstants {
  public static final int BOTTOM_ELEVATOR_CAN_ID;
  public static final int TOP_ELEVATOR_CAN_ID;
  public static final double ELEVATOR_BOTTOM_GEAR_RATIO;
  public static final double ELEVATOR_TOP_GEAR_RATIO;
  public static final double TOP_DRUM_RADIUS;
  public static final double BOTTOM_DRUM_RADIUS;

  public static final ElevatorSimParams ELEVATOR_SIM_PARAMS;

  public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;

  public static final DCMotor ELEVATOR_MOTOR_CONFIG;

  static {
    ELEVATOR_MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);
    GAINS =
        new Gains(
            new LoggedTunableNumber("Elevator/Gains/kP", 0.1),
            new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
            new LoggedTunableNumber("Elevator/Gains/kS", 0.0),
            new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
            new LoggedTunableNumber("Elevator/Gains/kA", 0.0));

    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Elevator/Max Acceleration", 4),
            new LoggedTunableNumber("Elevator/Goal Tolerance", 0.01));

    BOTTOM_ELEVATOR_CAN_ID = 0;
    TOP_ELEVATOR_CAN_ID = 0;
    ELEVATOR_BOTTOM_GEAR_RATIO = 1.0;
    ELEVATOR_TOP_GEAR_RATIO = 1.0;
    TOP_DRUM_RADIUS = 1;
    BOTTOM_DRUM_RADIUS = 1;

    ELEVATOR_SIM_PARAMS = new ElevatorSimParams(1, 1, 0, 0, 1, 1);

    ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;
  }

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static record Constraints(
      LoggedTunableNumber maxAcceleration, LoggedTunableNumber goalTolerance) {}

  public static record ElevatorSimParams(
      double TOP_CARRIAGE_MASS_KG,
      double BOTTOM_CARRIAGE_MASS_KG,
      double TOP_MIN_HEIGHT_METERS,
      double BOTTOM_MIN_HEIGHT_METERS,
      double TOP_MAX_HEIGHT_METERS,
      double BOTTOM_MAX_HEIGHT_METERS) {}

  @RequiredArgsConstructor
  public static enum ElevatorPositions {
    STOW(0.0, 0.0),
    INTAKE(0.0, 0.0),
    L1(0.0, 0.0),
    L2(0.0, 0.0),
    L3(0.0, 0.0),
    L4(0.0, 0.0);

    private final double topPosition;
    private final double bottomPosition;

    public double getTopPosition() {
      return topPosition;
    }

    public double getBottomPosition() {
      return bottomPosition;
    }
  }
}
