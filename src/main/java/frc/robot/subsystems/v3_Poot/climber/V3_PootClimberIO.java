package frc.robot.subsystems.v3_Poot.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V3_PootClimberIO {
  @AutoLog
  public static class V3_PootClimberIOInputs {
    public Rotation2d deploymentPosition = new Rotation2d();
    public double deploymentVelocityRadiansPerSecond = 0.0;
    public double deploymentAppliedVolts = 0.0;
    public double deploymentSupplyCurrentAmps = 0.0;
    public double deploymentTorqueCurrentAmps = 0.0;
    public double deploymentTemperatureCelsius = 0.0;

    public Rotation2d rollerPosition = new Rotation2d();
    public double rollerVelocityRadiansPerSecond = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;
  }
  /**
   * Updates the inputs for the manipulator subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(V3_PootClimberIOInputs inputs) {}

  /**
   * Sets the voltage for the intake.
   *
   * @param volts The voltage to set.
   */
  public default void setDeploymentVoltage(double volts) {}

  /**
   * Sets the voltage for the inner manipulator roller.
   *
   * @param volts The voltage to set.
   */
  public default void setRollerVoltage(double volts) {}

  /**
   * Gets the state of the climber based on the current draw.
   *
   * @return The state of the climber (climbed or not).
   */
  public default boolean isClimbed() {
    return false;
  }
}
