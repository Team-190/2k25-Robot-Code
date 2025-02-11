package frc.robot.subsystems.v0_funky.ground_coral;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V0_FunkyGroundCoralIO {
  @AutoLog
  public static class GoundCoralIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(GoundCoralIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
