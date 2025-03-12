package frc.robot.subsystems.v2_Delta.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V2_DeltaIntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Rotation2d position = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /**
   * Updates the inputs for the intake subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets the voltage for the intake.
   *
   * @param volts The voltage to set.
   */
  public default void setVoltage(double volts) {}
}
