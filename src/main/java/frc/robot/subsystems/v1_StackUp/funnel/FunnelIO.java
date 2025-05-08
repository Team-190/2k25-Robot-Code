package frc.robot.subsystems.v1_StackUp.funnel;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public Rotation2d clapDaddyPosition = new Rotation2d();
    public Rotation2d clapDaddyAbsolutePosition = new Rotation2d();
    public double clapDaddyVelocityRadiansPerSecond = 0.0;
    public double clapDaddyAppliedVolts = 0.0;
    public double clapDaddySupplyCurrentAmps = 0.0;
    public double clapDaddyTorqueCurrentAmps = 0.0;
    public double clapDaddyTemperatureCelsius = 0.0;
    public Rotation2d clapDaddyGoal = new Rotation2d();
    public Rotation2d clapDaddyPositionSetpoint = new Rotation2d();
    public Rotation2d clapDaddyPositionError = new Rotation2d();

    public Rotation2d rollerPosition = new Rotation2d();
    public double rollerVelocityRadiansPerSecond = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;

    public boolean hasCoral = false;
  }

  /**
   * Updates the inputs for the funnel subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(FunnelIOInputs inputs) {}

  /**
   * Sets the voltage for the clap daddy.
   *
   * @param volts The voltage to set.
   */
  public default void setClapDaddyVoltage(double volts) {}

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
  public default void setClapDaddyGoal(Rotation2d position) {}

  /** Stops the roller. */
  public default void stopRoller() {}

  /**
   * Checks if the clap daddy is at its goal position.
   *
   * @return True if the clap daddy is at its goal, false otherwise.
   */
  public default boolean atClapDaddyPositionGoal() {
    return false;
  }

  /**
   * Updates the control gains for the funnel subsystem.
   *
   * @param kP Proportional gain.
   * @param kD Derivative gain.
   * @param kS Static gain.
   * @param kV Velocity gain.
   * @param kA Acceleration gain.
   */
  public default void updateGains(double kP, double kD, double kS, double kV, double kA) {}

  /**
   * Updates the constraints for the funnel subsystem.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  public default void updateConstraints(double maxAcceleration, double maxVelocity) {}
}
