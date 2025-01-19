package frc.robot.subsystems.v1_gamma.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public double serializerPositionRadians = 0.0;
    public double serializerVelocityRadiansPerSecond = 0.0;
    public double serializerGoalRadians = 0.0;
    public double serializerAppliedVolts = 0.0;
    public double serializerSupplyCurrentAmps = 0.0;
    public double serializerTorqueCurrentAmps = 0.0;
    public double serializerTemperatureCelsius = 0.0;
    public double serializerPositionSetpointRadians = 0.0;
    public double serializerPositionErrorRadians = 0.0;
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

  public default void setSerializerVoltage(double volts) {}

  public default void setRollerVoltage(double volts) {}

  public default void setSerializerPosition(double radians) {}

  public default void setRollerVelocity(double radiansPerSecond) {}

  public default void stopRoller() {}

  public default boolean atSerializerGoal() {
    return false;
  }

  public default boolean atRollerGoal() {
    return false;
  }
}
