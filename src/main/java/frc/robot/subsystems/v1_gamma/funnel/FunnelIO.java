package frc.robot.subsystems.v1_gamma.funnel;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public Rotation2d serializerPosition = new Rotation2d();
    public Rotation2d serializerAbsolutePosition = new Rotation2d();
    public double serializerVelocityRadiansPerSecond = 0.0;
    public double serializerAppliedVolts = 0.0;
    public double serializerSupplyCurrentAmps = 0.0;
    public double serializerTorqueCurrentAmps = 0.0;
    public double serializerTemperatureCelsius = 0.0;
    public Rotation2d serializerGoal = new Rotation2d();
    public Rotation2d serializerPositionSetpoint = new Rotation2d();
    public Rotation2d serializerPositionError = new Rotation2d();

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
   * Sets the voltage for the serializer.
   *
   * @param volts The voltage to set.
   */
  public default void setSerializerVoltage(double volts) {}

  /**
   * Sets the voltage for the roller.
   *
   * @param volts The voltage to set.
   */
  public default void setRollerVoltage(double volts) {}

  /**
   * Sets the position for the serializer.
   *
   * @param position The position to set.
   */
  public default void setSerializerPosition(Rotation2d position) {}

  /** Stops the roller. */
  public default void stopRoller() {}

  /**
   * Checks if the serializer is at its goal position.
   *
   * @return True if the serializer is at its goal, false otherwise.
   */
  public default boolean atSerializerGoal() {
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
   * Updates the angle thresholds for the funnel subsystem.
   *
   * @param maxAngle The maximum angle of the serializer.
   * @param minAngle The minimum angle of the clapdaddy.
   */
  public default void updateThresholds(double maxAngle, double minAngle) {}

  /**
   * Updates the constraints for the funnel subsystem.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  public default void updateConstraints(double maxAcceleration, double maxVelocity) {}
}
