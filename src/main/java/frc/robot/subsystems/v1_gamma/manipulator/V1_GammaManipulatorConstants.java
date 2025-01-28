package frc.robot.subsystems.v1_gamma.manipulator;

import frc.robot.util.LoggedTunableNumber;

public class V1_GammaManipulatorConstants {
  public static final int MANIPULATOR_CAN_ID = 1;
  public static final int CORAL_SENSOR_ID = 2;
  public static final double SUPPLY_CURRENT_LIMIT = 40;
  public static final double MANIPULATOR_CURRENT_THRESHOLD = 0;
  public static final VoltageTunableNumbers VOLTAGES;

  static {
    VOLTAGES =
        new VoltageTunableNumbers(
            new LoggedTunableNumber("Manipulator/IntakeVolts", 12.0),
            new LoggedTunableNumber("Manipulator/ScoreVolts", -12.0));
  }

  public static final record VoltageTunableNumbers(
      LoggedTunableNumber INTAKE_VOLTS, LoggedTunableNumber SCORE_VOLTS) {}
}
