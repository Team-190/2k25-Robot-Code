package frc.robot.subsystems.v2_Redundancy.superstructure.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double extensionPositionMeters = 0.0;
    public double extensionVelocityMetersPerSecond = 0.0;
    public double extensionAppliedVolts = 0.0;
    public double extensionSupplyCurrentAmps = 0.0;
    public double extensionTorqueCurrentAmps = 0.0;
    public double extensionTemperatureCelsius = 0.0;
    public double extensionGoal = 0.0;
    public double extensionPositionSetpoint = 0.0;
    public double extensionPositionError = 0.0;

    public Rotation2d rollerPosition = new Rotation2d();
    public double rollerVelocityRadiansPerSecond = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;
  }

  /**
   * Updates the inputs for the intake subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets the voltage for the clap daddy.
   *
   * @param volts The voltage to set.
   */
  public default void setExtensionVoltage(double volts) {}

  /**
   * Sets the voltage for the roller.
   *
   * @param volts The voltage to set.
   */
  public default void setRollerVoltage(double volts) {}

  /**
   * Sets the goal for the clap daddy.
   *
   * @param position The position to set.
   */
  public default void setExtensionGoal(double position) {}

  /** Stops the roller. */
  public default void stopRoller() {}

  /**
   * Checks if the clap daddy is at its goal position.
   *
   * @return True if the clap daddy is at its goal, false otherwise.
   */
  public default boolean atExtensionPositionGoal() {
    return false;
  }

  /**
   * Updates the control gains for the intake subsystem.
   *
   * @param kP Proportional gain.
   * @param kD Derivative gain.
   * @param kS Static gain.
   * @param kV Velocity gain.
   * @param kA Acceleration gain.
   */
  public default void updateGains(double kP, double kD, double kS, double kV, double kA) {}

  /**
   * Updates the constraints for the intake subsystem.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  public default void updateConstraints(double maxAcceleration, double maxVelocity) {}

  public default void resetExtension() {}

  public default void maxExt() {}
}
