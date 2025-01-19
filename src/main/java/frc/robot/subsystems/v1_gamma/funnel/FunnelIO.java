package frc.robot.subsystems.v1_gamma.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public double clapperPositionRadians = 0.0;
    public double clapperVelocityRadiansPerSecond = 0.0;
    public double clapperGoalRadians = 0.0;
    public double clapperAppliedVolts = 0.0;
    public double clapperSupplyCurrentAmps = 0.0;
    public double clapperTorqueCurrentAmps = 0.0;
    public double clapperTemperatureCelsius = 0.0;
    public double clapperPositionSetpointRadians = 0.0;
    public double clapperPositionErrorRadians = 0.0;
    public double encoderPositionRadians = 0.0;

    public double rollerPositionRadians = 0.0;
    public double rollerVelocityRadiansPerSecond = 0.0;
    public double rollerGoalRadiansPerSecond = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;
    public double rollerVelocitySetpointRadiansPerSecond = 0.0;
    public double rollerVelocityErrorRadiansPerSecond = 0.0;

    public boolean hasCoral = false;
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void setClapperVoltage(double volts) {}

  public default void setRollerVoltage(double volts) {}

  public default void setClapperPosition(double radians) {}

  public default void setRollerVelocity(double radiansPerSecond) {}

  public default void stopRoller() {}

  public default boolean atClapperGoal() {
    return false;
  }

  public default boolean atRollerGoal() {
    return false;
  }
}
