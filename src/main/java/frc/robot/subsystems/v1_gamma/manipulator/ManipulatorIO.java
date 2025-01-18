package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ManipulatorIO {
  @AutoLog
  public static class ManipulatorIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public boolean manipulatorHasCoral = false;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

}
