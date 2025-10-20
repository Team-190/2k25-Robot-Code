package frc.robot.subsystems.shared.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.util.LoggedTunableNumber;
import java.util.Map;
import lombok.RequiredArgsConstructor;

public class ElevatorConstants {
  public static final int ELEVATOR_CAN_ID;
  public static final double ELEVATOR_GEAR_RATIO;
  public static final double DRUM_RADIUS;

  public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT;
  public static final double ELEVATOR_STATOR_CURRENT_LIMIT;

  public static final ElevatorParameters ELEVATOR_PARAMETERS;
  public static final Gains GAINS;
  public static final Constraints CONSTRAINTS;
  public static final Gains STOW_GAINS;
  public static final Constraints STOW_CONSTRAINTS;

  public static final Map<ReefState, ElevatorPositions> REEF_STATE_ELEVATOR_POSITION_MAP;

  static {
    REEF_STATE_ELEVATOR_POSITION_MAP =
        Map.ofEntries(
            Map.entry(ReefState.STOW, ElevatorPositions.STOW),
            Map.entry(ReefState.POST_PROCESSOR, ElevatorPositions.POST_PROCESSOR),
            Map.entry(ReefState.HIGH_STOW, ElevatorPositions.HIGH_STOW),
            Map.entry(ReefState.CORAL_INTAKE, ElevatorPositions.CORAL_INTAKE),
            Map.entry(ReefState.ALGAE_FLOOR_INTAKE, ElevatorPositions.ALGAE_INTAKE),
            Map.entry(ReefState.ALGAE_MID, ElevatorPositions.ALGAE_MID),
            Map.entry(ReefState.HANDOFF, ElevatorPositions.HANDOFF),
            Map.entry(ReefState.ALGAE_INTAKE_TOP, ElevatorPositions.ALGAE_INTAKE_TOP),
            Map.entry(ReefState.ALGAE_INTAKE_BOTTOM, ElevatorPositions.ALGAE_INTAKE_BOT),
            Map.entry(ReefState.L1, ElevatorPositions.L1),
            Map.entry(ReefState.L2, ElevatorPositions.L2),
            Map.entry(ReefState.L3, ElevatorPositions.L3),
            Map.entry(ReefState.L4, ElevatorPositions.L4),
            Map.entry(ReefState.L4_PLUS, ElevatorPositions.L4_PLUS),
            Map.entry(ReefState.ALGAE_SCORE, ElevatorPositions.ALGAE_SCORE));

    switch (Constants.ROBOT) {
      case V1_STACKUP:
      case V1_STACKUP_SIM:
        ELEVATOR_CAN_ID = 20;
        ELEVATOR_GEAR_RATIO = 3.0;
        DRUM_RADIUS = Units.inchesToMeters(2.256 / 2.0);

        ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;
        ELEVATOR_STATOR_CURRENT_LIMIT = 40;

        ELEVATOR_PARAMETERS =
            new ElevatorParameters(
                DCMotor.getKrakenX60Foc(4), 6.803886, 0.0, 1.43 + Units.inchesToMeters(0.5), 4);

        switch (Constants.getMode()) {
          case REAL:
          case REPLAY:
          default:
            GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Gains/kP", 67.000000),
                    new LoggedTunableNumber("Elevator/Gains/kD", 0.1),
                    new LoggedTunableNumber("Elevator/Gains/kS", 0.225),
                    new LoggedTunableNumber("Elevator/Gains/kG", 0.075),
                    new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
            CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Max Acceleration", 190.0),
                    new LoggedTunableNumber("Elevator/Cruising Velocity", 41.0),
                    new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
            STOW_GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Gains/kP", 67.000000),
                    new LoggedTunableNumber("Elevator/Gains/kD", 0.1),
                    new LoggedTunableNumber("Elevator/Gains/kS", 0.225),
                    new LoggedTunableNumber("Elevator/Gains/kG", 0.075),
                    new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
            STOW_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Max Acceleration", 16.0),
                    new LoggedTunableNumber("Elevator/Cruising Velocity", 16.0),
                    new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
            break;
          case SIM:
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
            STOW_GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Gains/kP", 20.0),
                    new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kS", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kG", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
            STOW_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Max Acceleration", 101.078594),
                    new LoggedTunableNumber("Elevator/Cruising Velocity", 11.329982),
                    new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
            break;
        }
        break;

      case V3_EPSILON:
      case V3_EPSILON_SIM:
        ELEVATOR_CAN_ID = 20;
        ELEVATOR_GEAR_RATIO = 4.0;
        DRUM_RADIUS = Units.inchesToMeters(2.211 / 2.0);

        ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;
        ELEVATOR_STATOR_CURRENT_LIMIT = 80;

        ELEVATOR_PARAMETERS =
            new ElevatorParameters(
                DCMotor.getKrakenX60Foc(2), 6.803886, 0.0, Units.inchesToMeters(53), 2);

        switch (Constants.getMode()) {
          case REAL:
          case REPLAY:
          default:
            GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Gains/kP", 5),
                    new LoggedTunableNumber("Elevator/Gains/kD", 0),
                    new LoggedTunableNumber("Elevator/Gains/kS", 0.070776),
                    new LoggedTunableNumber("Elevator/Gains/kG", 0.35521),
                    new LoggedTunableNumber("Elevator/Gains/kV", 0),
                    new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
            CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Max Acceleration", 11),
                    new LoggedTunableNumber("Elevator/Cruising Velocity", 5),
                    new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
            STOW_GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Stow Gains/kP", 5),
                    new LoggedTunableNumber("Elevator/Stow Gains/kD", 0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kS", 0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kG", 0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kA", 0.0));
            STOW_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Stow Max Acceleration", 11),
                    new LoggedTunableNumber("Elevator/Stow Cruising Velocity", 5),
                    new LoggedTunableNumber("Elevator/Stow Goal Tolerance", 0.02));
            break;
          case SIM:
            GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Gains/kP", 50.0),
                    new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kS", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kG", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
            CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Max Acceleration", 201.078594),
                    new LoggedTunableNumber("Elevator/Cruising Velocity", 21.329982),
                    new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
            STOW_GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Stow Gains/kP", 20.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kD", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kS", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kG", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kA", 0.0));
            STOW_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Stow Max Acceleration", 101.078594),
                    new LoggedTunableNumber("Elevator/Stow Cruising Velocity", 11.329982),
                    new LoggedTunableNumber("Elevator/Stow Goal Tolerance", 0.02));
            break;
        }
        break;

      case V2_REDUNDANCY:
      case V2_REDUNDANCY_SIM:
      default:
        ELEVATOR_CAN_ID = 20;
        ELEVATOR_GEAR_RATIO = 4.0;
        DRUM_RADIUS = Units.inchesToMeters(2.256 / 2.0);

        ELEVATOR_SUPPLY_CURRENT_LIMIT = 40;
        ELEVATOR_STATOR_CURRENT_LIMIT = 80;

        ELEVATOR_PARAMETERS =
            new ElevatorParameters(
                DCMotor.getKrakenX60Foc(2), 6.803886, 0.0, 1.43 + Units.inchesToMeters(0.5), 2);

        switch (Constants.getMode()) {
          case REAL:
          case REPLAY:
          default:
            GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Gains/kP", 1),
                    new LoggedTunableNumber("Elevator/Gains/kD", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kS", 0.225),
                    new LoggedTunableNumber("Elevator/Gains/kG", 0.075),
                    new LoggedTunableNumber("Elevator/Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Gains/kA", 0.0));
            CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Max Acceleration", 1),
                    new LoggedTunableNumber("Elevator/Cruising Velocity", 1),
                    new LoggedTunableNumber("Elevator/Goal Tolerance", 0.02));
            STOW_GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Stow Gains/kP", 2.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kD", 0.1),
                    new LoggedTunableNumber("Elevator/Stow Gains/kS", 0.225),
                    new LoggedTunableNumber("Elevator/Stow Gains/kG", 0.075),
                    new LoggedTunableNumber("Elevator/Stow Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kA", 0.0));
            STOW_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Stow Max Acceleration", 16.0),
                    new LoggedTunableNumber("Elevator/Stow Cruising Velocity", 16.0),
                    new LoggedTunableNumber("Elevator/Stow Goal Tolerance", 0.02));
            break;
          case SIM:
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
            STOW_GAINS =
                new Gains(
                    new LoggedTunableNumber("Elevator/Stow Gains/kP", 20.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kD", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kS", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kG", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kV", 0.0),
                    new LoggedTunableNumber("Elevator/Stow Gains/kA", 0.0));
            STOW_CONSTRAINTS =
                new Constraints(
                    new LoggedTunableNumber("Elevator/Stow Max Acceleration", 101.078594),
                    new LoggedTunableNumber("Elevator/Stow Cruising Velocity", 11.329982),
                    new LoggedTunableNumber("Elevator/Stow Goal Tolerance", 0.02));
            break;
        }
        break;
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

  public static record PositionConstants(double V1, double V2, double V3) {}

  @RequiredArgsConstructor
  public static enum ElevatorPositions {
    STOW(new PositionConstants(0.0, 0.0, 0.0)),
    POST_PROCESSOR(new PositionConstants(0.5, 0.5, 0.5)),
    CORAL_INTAKE(new PositionConstants(0.0, 0.0, Units.inchesToMeters(34.85))),
    ALGAE_INTAKE(
        new PositionConstants(
            0.2161583093038944 + Units.inchesToMeters(1),
            0.2161583093038944 + Units.inchesToMeters(1),
            0)),
    ALGAE_MID(
        new PositionConstants(
            0.7073684509805078, 0.7073684509805078, 1.2)), // USED AS PRE-HANDOFF FOR V3
    ALGAE_INTAKE_TOP(
        new PositionConstants(
            1.17 - Units.inchesToMeters(8),
            1.17 - Units.inchesToMeters(8),
            Units.inchesToMeters(40))),
    ALGAE_INTAKE_BOT(
        new PositionConstants(
            0.79 - Units.inchesToMeters(8),
            0.79 - Units.inchesToMeters(8),
            Units.inchesToMeters(25))),
    ASS_TOP(new PositionConstants(1.2, 0.0, 0.0)),
    ASS_BOT(new PositionConstants(0.82, 0.0, 0.0)),
    L1(
        new PositionConstants(
            0.11295250319916351, 0.11295250319916351, Units.inchesToMeters(34.75))),
    L2(new PositionConstants(0.37296301250898894, 0.37296301250898894, Units.inchesToMeters(12.5))),
    L3(
        new PositionConstants(
            0.7606347556550676 + Units.inchesToMeters(1.0),
            0.7606347556550676 + Units.inchesToMeters(1.0),
            Units.inchesToMeters(12.5 + 15.87))),
    L4(
        new PositionConstants(
            1.3864590139769697 + Units.inchesToMeters(0.5),
            1.3864590139769697 + Units.inchesToMeters(0.5),
            Units.inchesToMeters(48))),
    L4_PLUS(
        new PositionConstants(
            0.0,
            1.3864590139769697 + Units.inchesToMeters(2.0),
            0.0)), // DOES NOT EXIST FOR V3 AND V1
    ALGAE_SCORE(
        new PositionConstants(
            1.3864590139769697 + Units.inchesToMeters(0.5),
            1.3864590139769697 + Units.inchesToMeters(0.5),
            Units.inchesToMeters(50))),

    HIGH_STOW(new PositionConstants(0, 0, 0.5)),
    HANDOFF(new PositionConstants(0, 0, Units.inchesToMeters(33.25))),
    ;

    private final PositionConstants position;

    public double getPosition() {
      switch (Constants.ROBOT) {
        case V1_STACKUP, V1_STACKUP_SIM:
          return position.V1();
        case V2_REDUNDANCY, V2_REDUNDANCY_SIM:
          return position.V2();
        case V3_EPSILON, V3_EPSILON_SIM:
          return position.V3();
        default:
          return position.V3();
      }
    }

    public static ElevatorPositions getPosition(ReefState state) {
      for (ElevatorPositions pos : values()) {
        if (REEF_STATE_ELEVATOR_POSITION_MAP.get(state) == pos) {
          return pos;
        }
      }
      return null;
    }
  }
}
