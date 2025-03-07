package frc.robot.subsystems.v1_StackUp.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableNumber;

public class V1_StackUpManipulatorConstants {
  public static final int MANIPULATOR_CAN_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double MANIPULATOR_CURRENT_THRESHOLD;
  public static final Rotation2d MANIPULATOR_TOGGLE_ARM_ROTATION;
  public static final Voltages VOLTAGES;

  static {
    MANIPULATOR_CAN_ID = 30;
    SUPPLY_CURRENT_LIMIT = 40.0;
    MANIPULATOR_CURRENT_THRESHOLD = 39.0;
    MANIPULATOR_TOGGLE_ARM_ROTATION = Rotation2d.fromRadians(10);
    VOLTAGES =
        new Voltages(
            new LoggedTunableNumber("Manipulator/Intake Volts", 6.0),
            new LoggedTunableNumber("Manipulator/Score Volts", 4.0),
            new LoggedTunableNumber("Manipulator/Remove Algae Volts", 12),
            new LoggedTunableNumber("Manipulator/HalfScore Volts", 1.0),
            new LoggedTunableNumber("Manipulator/L1 Volts", 3.5));
  }

  public static final record Voltages(
      LoggedTunableNumber INTAKE_VOLTS,
      LoggedTunableNumber SCORE_VOLTS,
      LoggedTunableNumber REMOVE_ALGAE,
      LoggedTunableNumber HALF_VOLTS,
      LoggedTunableNumber L1_VOLTS) {}
}
