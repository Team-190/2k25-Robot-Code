package frc.robot.subsystems.v3_Epsilon.superstructure.manipulator;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class V3_EpsilonManipulatorConstants {
  public static final ArmParameters ARM_PARAMETERS;
  public static final Gains EMPTY_GAINS;
  public static final Gains CORAL_GAINS;
  public static final Gains ALGAE_GAINS;
  public static final Constraints CONSTRAINTS;

  public static final int ROLLER_CAN_ID;
  public static final double ROLLER_CURRENT_THRESHOLD;
  public static final Rotation2d ROLLER_TOGGLE_ARM_ROTATION;
  public static final Voltages ROLLER_VOLTAGES;

  public static final ManipulatorCurrentLimits CURRENT_LIMITS;

  public static final int ARM_CAN_ID = 42;
  public static final Rotation2d TOGGLE_ARM_ROTATION;

  public static final int CAN_RANGE_ID = 41;

  public static final double ALGAE_CAN_RANGE_THRESHOLD = 0.5;
  public static final double CORAL_CAN_RANGE_THRESHOLD = 0.5;

  static {
    ARM_PARAMETERS = new ArmParameters(DCMotor.getKrakenX60Foc(1), 1, 90.0, .618);
    EMPTY_GAINS =
        new Gains(
            new LoggedTunableNumber("Manipulator/Arm/Empty/kP", 50),
            new LoggedTunableNumber("Manipulator/Arm/Empty/kD", 0),
            new LoggedTunableNumber("Manipulator/Arm/Empty/kS", 0.24274),
            new LoggedTunableNumber("Manipulator/Arm/Empty/kG", 0.66177),
            new LoggedTunableNumber("Manipulator/Arm/Empty/kV", 0),
            new LoggedTunableNumber("Manipulator/Arm/Empty/kA", 0));
    CORAL_GAINS =
        new Gains(
            new LoggedTunableNumber("Manipulator/ArmWithoutAlgae/kP", 125),
            new LoggedTunableNumber("Manipulator/ArmWithoutAlgae/kD", 0),
            new LoggedTunableNumber("Manipulator/ArmWithoutAlgae/kS", 0.24274),
            new LoggedTunableNumber("Manipulator/ArmWithoutAlgae/kG", 0.66177),
            new LoggedTunableNumber("Manipulator/ArmWithoutAlgae/kV", 0.0),
            new LoggedTunableNumber("Manipulator/ArmWithoutAlgae/kA", 0.0));
    ALGAE_GAINS =
        new Gains(
            new LoggedTunableNumber("Manipulator/ArmWithAlgae/kP", 125),
            new LoggedTunableNumber("Manipulator/ArmWithAlgae/kD", 0),
            new LoggedTunableNumber("Manipulator/ArmWithAlgae/kS", 0.65347),
            new LoggedTunableNumber("Manipulator/ArmWithAlgae/kG", 2.0762),
            new LoggedTunableNumber("Manipulator/ArmWithAlgae/kV", 0.0),
            new LoggedTunableNumber("Manipulator/ArmWithAlgae/kA", 0.0));
    CONSTRAINTS =
        new Constraints(
            new LoggedTunableNumber("Manipulator/Arm/MaxAcceleration", 20.0),
            new LoggedTunableNumber("Manipulator/Arm/CruisingVelocity", 50.0),
            new LoggedTunableNumber("Manipulator/Arm/GoalTolerance", Units.degreesToRadians(3)));

    ROLLER_CAN_ID = 30;
    ROLLER_CURRENT_THRESHOLD = 60.0;
    ROLLER_TOGGLE_ARM_ROTATION = Rotation2d.fromRadians(10);
    ROLLER_VOLTAGES =
        new Voltages(
            new LoggedTunableNumber("Manipulator/Coral Intake Volts", 6.0),
            new LoggedTunableNumber("Manipulator/Algae Intake Volts", 12.0),
            new LoggedTunableNumber("Manipulator/L4 Volts", 4.6 * 1.56),
            new LoggedTunableNumber("Manipulator/Score Coral Volts", 4.8 * 1.56),
            new LoggedTunableNumber("Manipulator/Score Algae Volts", -6),
            new LoggedTunableNumber("Manipulator/Remove Algae Volts", 12),
            new LoggedTunableNumber("Manipulator/HalfScore Volts", 1.0 * 1.56),
            new LoggedTunableNumber("Manipulator/L1 Volts", 3.5 * 1.56));

    CURRENT_LIMITS = new ManipulatorCurrentLimits(40, 40, 40, 40);

    TOGGLE_ARM_ROTATION = new Rotation2d();
  }

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {
    public SlotConfigs toTalonFXSlotConfigs() {
      return new SlotConfigs()
          .withKP(kP.get())
          .withKD(kD.get())
          .withKS(kS.get())
          .withKG(kG.get())
          .withKV(kV.get())
          .withKA(kA.get())
          .withGravityType(GravityTypeValue.Arm_Cosine);
    }
  }

  public static final record ManipulatorCurrentLimits(
      double MANIPULATOR_SUPPLY_CURRENT_LIMIT,
      double ROLLER_SUPPLY_CURRENT_LIMIT,
      double MANIPULATOR_STATOR_CURRENT_LIMIT,
      double ROLLER_STATOR_CURRENT_LIMIT) {}

  public static record Constraints(
      LoggedTunableNumber maxAccelerationRotationsPerSecondSquared,
      LoggedTunableNumber cruisingVelocityRotationsPerSecond,
      LoggedTunableNumber goalToleranceRadians) {}

  public static final record Voltages(
      LoggedTunableNumber CORAL_INTAKE_VOLTS,
      LoggedTunableNumber ALGAE_INTAKE_VOLTS,
      LoggedTunableNumber L4_VOLTS,
      LoggedTunableNumber SCORE_CORAL_VOLTS,
      LoggedTunableNumber SCORE_ALGAE_VOLTS,
      LoggedTunableNumber REMOVE_ALGAE,
      LoggedTunableNumber HALF_VOLTS,
      LoggedTunableNumber L1_VOLTS) {}

  public static record ArmParameters(
      DCMotor MOTOR_CONFIG, int NUM_MOTORS, double GEAR_RATIO, double LENGTH_METERS) {}

  @RequiredArgsConstructor
  public static enum ManipulatorArmState {
    PRE_SCORE(Rotation2d.fromDegrees(50.0)),
    SCORE(Rotation2d.fromDegrees(55.0)), // Placeholder value. Make sure to test
    PROCESSOR(Rotation2d.fromDegrees(41.279296875)),
    REEF_INTAKE(Rotation2d.fromDegrees(46.279296875)),
    INTAKE_OUT_LINE(Rotation2d.fromDegrees(61)),
    FLOOR_INTAKE(Rotation2d.fromDegrees(73.5)),
    STOW_LINE(Rotation2d.fromDegrees(75)),
    TRANSITION(Rotation2d.fromDegrees(15.0)), // Placeholder value. Make sure to test
    VERTICAL_UP(Rotation2d.fromDegrees(0)),
    HANDOFF(Rotation2d.kPi),
    SAFE_ANGLE(Rotation2d.fromDegrees(150));

    private final Rotation2d angle;

    public Rotation2d getAngle(Side side) {
      if (side == Side.NEGATIVE) {
        return angle.unaryMinus();
      }
      return angle;
    }
  }

  // Will add more states later
  public static enum ManipulatorRollerState {
    STOP(0.0),
    CORAL_INTAKE(6.0),
    ALGAE_INTAKE(12.0),
    L4_SCORE(4.6 * 1.56),
    SCORE_CORAL(4.8 * 1.56),
    SCORE_ALGAE(-6),
    REMOVE_ALGAE(-12),
    L1_SCORE(3.5 * 1.56);

    private final double voltage;

    ManipulatorRollerState(double voltage) {
      this.voltage = voltage;
    }

    public double getVoltage() {
      return voltage;
    }
  }

  public enum Side {
    POSITIVE,
    NEGATIVE
  }
}
