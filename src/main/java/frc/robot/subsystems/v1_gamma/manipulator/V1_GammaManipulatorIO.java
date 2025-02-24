package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V1_GammaManipulatorIO {
  @AutoLog
  public static class ManipulatorIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /**
   * Updates the inputs for the manipulator subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(ManipulatorIOInputs inputs) {}

  /**
   * Sets the voltage for the manipulator.
   *
   * @param volts The voltage to set.
   */
  public default void setVoltage(double volts) {}
}
