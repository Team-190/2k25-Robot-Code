package frc.robot.subsystems.v3_Epsilon.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V3_EpsilonManipulatorIO {

  @AutoLog
  public static class ManipulatorIOInputs {
    public Rotation2d armPosition = new Rotation2d();
    public double armVelocityRadiansPerSecond = 0.0;
    public double armAppliedVolts = 0.0;
    public double armSupplyCurrentAmps = 0.0;
    public double armTorqueCurrentAmps = 0.0;
    public double armTemperatureCelsius = 0.0;

    public Rotation2d armPositionGoal = new Rotation2d();
    public Rotation2d armPositionSetpoint = new Rotation2d();
    public Rotation2d armPositionError = new Rotation2d();

    public Rotation2d rollerPosition = new Rotation2d();
    public double rollerVelocityRadiansPerSecond = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;

    public double canRangeDistanceMeters = 0.0;
  }

  /**
   * Updates the inputs for the manipulator subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(ManipulatorIOInputs inputs) {}

  /**
   * Sets the voltage for the manipulator.
   *
   * @param volts The voltage to set.
   */
  public default void setPivotVoltage(double volts) {}

  public default void setRollerVoltage(double volts) {}

  public default void setPivotGoal(Rotation2d rotation) {}

  public default void setManipulatorState(V3_EpsilonManipulatorConstants.PivotState state) {}

  /**
   * Sets the gains for the arm.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  public default void updateSlot0ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  public default void updateSlot1ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  public default void updateSlot2ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  /**
   * Sets the constraints for the arm.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  public default void updateArmConstraints(double maxAcceleration, double cruisingVelocity) {}

  public default void setSlot(int slot) {}
}
