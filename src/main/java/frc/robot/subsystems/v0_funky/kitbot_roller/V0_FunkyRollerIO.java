package frc.robot.subsystems.v0_funky.kitbot_roller;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V0_FunkyRollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
