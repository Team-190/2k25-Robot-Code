package frc.robot.subsystems.v2_Redundancy.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableNumber;

public class V2_RedundancyManipulatorConstants {
  public static final int MANIPULATOR_CAN_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double MANIPULATOR_CURRENT_THRESHOLD;
  public static final Rotation2d MANIPULATOR_TOGGLE_ARM_ROTATION;
  public static final Voltages VOLTAGES;

  static {
    MANIPULATOR_CAN_ID = 30;
    SUPPLY_CURRENT_LIMIT = 20.0;
    MANIPULATOR_CURRENT_THRESHOLD = 60.0;
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
