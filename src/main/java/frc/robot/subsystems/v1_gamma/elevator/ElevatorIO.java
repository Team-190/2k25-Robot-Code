package frc.robot.subsystems.v1_gamma.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double[] positionMeters = {0.0, 0.0, 0.0, 0.0};
    public double[] velocityMetersPerSecond = {0.0, 0.0, 0.0, 0.0};
    public double[] appliedVolts = {0.0, 0.0, 0.0, 0.0};
    public double[] supplyCurrentAmps = {0.0, 0.0, 0.0, 0.0};
    public double[] torqueCurrentAmps = {0.0, 0.0, 0.0, 0.0};
    public double[] temperatureCelsius = {0.0, 0.0, 0.0, 0.0};
    public double[] positionSetpointMeters = {0.0, 0.0, 0.0, 0.0};
    public double[] positionErrorMeters = {0.0, 0.0, 0.0, 0.0};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setCurrent(double amps) {}

  public default void setPosition(double meters) {}

  public default void setPositionGoal(double meters) {}
}
