package frc.robot.subsystems.v1_StackUp.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V1_StackUpManipulatorIO {
  @AutoLog
  public static class V1_StackUpManipulatorIOInputs {
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
  public default void updateInputs(V1_StackUpManipulatorIOInputs inputs) {}

  /**
   * Sets the voltage for the manipulator.
   *
   * @param volts The voltage to set.
   */
  public default void setVoltage(double volts) {}
}
