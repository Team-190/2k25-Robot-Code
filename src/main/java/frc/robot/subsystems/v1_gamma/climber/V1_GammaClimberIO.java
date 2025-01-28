package frc.robot.subsystems.v1_gamma.climber;

import org.littletonrobotics.junction.AutoLog;

public interface V1_GammaClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotationsPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /**
   * Updates the inputs for the climber.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Sets the voltage for the climber.
   *
   * @param volts The voltage to set.
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the current for the climber.
   *
   * @param amps The current to set.
   */
  public default void setCurrent(double amps) {}

  /**
   * Gets the state of the climber based on the current draw.
   *
   * @return The state of the climber (climbed or not).
   */
  public default boolean isClimbed() {
    return false;
  }
}
