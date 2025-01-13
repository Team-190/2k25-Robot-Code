package frc.robot.subsystems.v1_gamma.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double topPositionMeters = 0.0;
    public double topVelocityMetersPerSecond = 0.0;
    public double topAppliedVolts = 0.0;
    public double topSupplyCurrentAmps = 0.0;
    public double topTorqueCurrentAmps = 0.0;
    public double topTemperatureCelcius = 0.0;
    public double topPositionSetpointMeters = 0.0;
    public double topPositionErrorMeters = 0.0;

    public double bottomPositionMeters = 0.0;
    public double bottomVelocityMetersPerSecond = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomSupplyCurrentAmps = 0.0;
    public double bottomTorqueCurrentAmps = 0.0;
    public double bottomTemperatureCelcius = 0.0;
    public double bottomPositionSetpointMeters = 0.0;
    public double bottomPositionErrorMeters = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setBottomVoltage(double volts) {}

  public default void setTopVoltage(double volts) {}

  public default void setBottomPosition(double meters) {}

  public default void setTopPosition(double meters) {}

  public default void setBottomPositionGoal(double meters) {}

  public default void setTopPositionGoal(double meters) {}
}
