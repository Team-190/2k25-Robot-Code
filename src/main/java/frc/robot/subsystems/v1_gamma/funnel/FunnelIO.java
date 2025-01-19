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

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void setSerializerVoltage(double volts) {}

  public default void setRollerVoltage(double volts) {}

  public default void setSerializerPosition(Rotation2d position) {}

  public default void stopRoller() {}

  public default boolean atSerializerGoal() {
    return false;
  }
}
