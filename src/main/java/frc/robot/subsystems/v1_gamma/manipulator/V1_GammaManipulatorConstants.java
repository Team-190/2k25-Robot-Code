package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableNumber;

public class V1_GammaManipulatorConstants {
  public static final int MANIPULATOR_CAN_ID;
  public static final double SUPPLY_CURRENT_LIMIT;
  public static final double MANIPULATOR_CURRENT_THRESHOLD;
  public static final Rotation2d MANIPULATOR_STOW_ROTATIONS;
  public static final Rotation2d MANIPULATOR_DEPLOY_ROTATIONS;
  public static final Voltages VOLTAGES;

  static {
    MANIPULATOR_CAN_ID = 30;
    SUPPLY_CURRENT_LIMIT = 40.0;
    MANIPULATOR_CURRENT_THRESHOLD = 30.0;
    MANIPULATOR_STOW_ROTATIONS = Rotation2d.fromRadians(10);
    MANIPULATOR_DEPLOY_ROTATIONS = Rotation2d.fromRadians(1);

    VOLTAGES =
        new Voltages(
            new LoggedTunableNumber("Manipulator/Intake Volts", 6.0),
            new LoggedTunableNumber("Manipulator/Score Volts", 4.0),
            new LoggedTunableNumber("Manipulator/Remove Algae Volts", 12),
            new LoggedTunableNumber("Manipulator/HalfScore Volts", 1.0));
  }

  public static final record Voltages(
      LoggedTunableNumber INTAKE_VOLTS,
      LoggedTunableNumber SCORE_VOLTS,
      LoggedTunableNumber REMOVE_ALGAE,
      LoggedTunableNumber HALF_VOLTS) {}
}
