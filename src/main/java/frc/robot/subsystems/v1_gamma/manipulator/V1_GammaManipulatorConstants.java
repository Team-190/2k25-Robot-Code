package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableNumber;

public class V1_GammaManipulatorConstants {
  public static final int MANIPULATOR_CAN_ID;
  public static final int CORAL_SENSOR_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double MANIPULATOR_CURRENT_THRESHOLD;
  public static final Rotation2d halfScoreRotation;
  public static final Voltages VOLTAGES;

  static {
    MANIPULATOR_CAN_ID = 1;
    CORAL_SENSOR_ID = 2;
    SUPPLY_CURRENT_LIMIT = 40.0;
    MANIPULATOR_CURRENT_THRESHOLD = 0.0;
    halfScoreRotation = new Rotation2d();
    VOLTAGES =
        new Voltages(
            new LoggedTunableNumber("Manipulator/Intake Volts", 12.0),
            new LoggedTunableNumber("Manipulator/Score Volts", -12.0));
  }

  public static final record Voltages(
      LoggedTunableNumber INTAKE_VOLTS, LoggedTunableNumber SCORE_VOLTS) {}
}
