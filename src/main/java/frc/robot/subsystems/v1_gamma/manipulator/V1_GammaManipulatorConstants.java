package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableNumber;

public class V1_GammaManipulatorConstants {
  public static final int MANIPULATOR_CAN_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double MANIPULATOR_CURRENT_THRESHOLD;
  public static final Rotation2d halfScoreRotation;
  public static final Rotation2d halfUnScoreRotation;
  public static final Voltages VOLTAGES;

  static {
    MANIPULATOR_CAN_ID = 30;
    SUPPLY_CURRENT_LIMIT = 40.0;
    MANIPULATOR_CURRENT_THRESHOLD = 30.0;
    halfScoreRotation = Rotation2d.fromRotations(1.4);
    halfUnScoreRotation = Rotation2d.fromRotations(.25);
    VOLTAGES =
        new Voltages(
            new LoggedTunableNumber("Manipulator/Intake Volts", 6.0),
            new LoggedTunableNumber("Manipulator/Score Volts", 6.0),
            new LoggedTunableNumber("Manipulator/Score L4 Volts", 3.0),
            new LoggedTunableNumber("Manipulator/HalfScore Volts", 1.0));
  }

  public static final record Voltages(
      LoggedTunableNumber INTAKE_VOLTS,
      LoggedTunableNumber SCORE_VOLTS,
      LoggedTunableNumber SCORE_L4_VOLTS,
      LoggedTunableNumber HALF_VOLTS) {}
}
