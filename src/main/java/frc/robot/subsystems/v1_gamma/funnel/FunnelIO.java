package frc.robot.subsystems.v1_gamma.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public double crabPositionRadians = 0.0;
    public double crabVelocityRadiansPerSecond = 0.0;
    public double crabGoalRadians = 0.0;
    public double crabAppliedVolts = 0.0;
    public double crabSupplyCurrentAmps = 0.0;
    public double crabTorqueCurrentAmps = 0.0;
    public double crabTemperatureCelsius = 0.0;
    public double crabPositionSetpointRadians = 0.0;
    public double crabPositionErrorRadians = 0.0;
    public double encoderPositionRadians = 0.0;

    public double intakePositionRadians = 0.0;
    public double intakeVelocityRadiansPerSecond = 0.0;
    public double intakeGoalRadiansPerSecond = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public double intakeTorqueCurrentAmps = 0.0;
    public double intakeTemperatureCelsius = 0.0;
    public double intakeVelocitySetpointRadiansPerSecond = 0.0;
    public double intakeVelocityErrorRadiansPerSecond = 0.0;

    public boolean hasCoral = false;
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void setCrabVoltage(double volts) {}

  public default void setIntakeVoltage(double volts) {}

  public default void setCrabPosition(double radians) {}

  public default void setIntakeVelocity(double radiansPerSecond) {}

  public default void stopIntake() {}

  public default boolean atCrabGoal() {
    return false;
  }

  public default boolean atIntakeGoal() {
    return false;
  }
}
