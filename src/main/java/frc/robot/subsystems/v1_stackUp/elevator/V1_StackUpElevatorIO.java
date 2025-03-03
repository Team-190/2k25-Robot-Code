package frc.robot.subsystems.v1_stackUp.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface V1_StackUpElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSecond = 0.0;
    public double[] appliedVolts = {0.0, 0.0, 0.0, 0.0};
    public double[] supplyCurrentAmps = {0.0, 0.0, 0.0, 0.0};
    public double[] torqueCurrentAmps = {0.0, 0.0, 0.0, 0.0};
    public double[] temperatureCelsius = {0.0, 0.0, 0.0, 0.0};
    public double positionGoalMeters = 0.0;
    public double positionSetpointMeters = 0.0;
    public double positionErrorMeters = 0.0;
  }

  /**
   * Updates the inputs for the elevator.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the voltage for the elevator.
   *
   * @param volts The voltage to set.
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the position for the elevator.
   *
   * @param meters The position to set in meters.
   */
  public default void setPosition(double meters) {}

  /**
   * Sets the position goal for the elevator.
   *
   * @param meters The position goal to set in meters.
   */
  public default void setPositionGoal(double meters) {}

  /**
   * Sets the gains for the elevator.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  public default void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  /**
   * Sets the constraints for the elevator.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  public default void updateConstraints(double maxAcceleration, double cruisingVelocity) {}
}
