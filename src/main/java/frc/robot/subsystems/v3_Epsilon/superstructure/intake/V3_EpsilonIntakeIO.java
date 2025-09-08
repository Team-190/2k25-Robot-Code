package frc.robot.subsystems.v3_Epsilon.superstructure.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V3_EpsilonIntakeIO {
  @AutoLog
  public static class V3_EpsilonIntakeIOInputs {
    public Rotation2d pivotPosition = new Rotation2d();
    public double pivotVelocityRadiansPerSecond = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotTorqueCurrentAmps = 0.0;
    public double pivotTemperatureCelsius = 0.0;

    public Rotation2d pivotPositionGoal = new Rotation2d();
    public Rotation2d pivotPositionSetpoint = new Rotation2d();
    public Rotation2d pivotPositionError = new Rotation2d();

    public Rotation2d rollerPosition = new Rotation2d();
    public double rollerVelocityRadiansPerSecond = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerTorqueCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;

    public boolean leftHasCoral =
        false; // Left and Right based on the robot's perspective with intake at the front
    public boolean rightHasCoral = false;
  }

  /**
   * Updates the inputs for the manipulator subsystem.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(V3_EpsilonIntakeIOInputs inputs) {}

  /**
   * Sets the voltage for the intake.
   *
   * @param volts The voltage to set.
   */
  public default void setPivotVoltage(double volts) {}

  /**
   * Sets the voltage for the manipulator.
   *
   * @param volts The voltage to set.
   */
  public default void setRollerVoltage(double volts) {}

  /**
   * Sets the position goal for the intake.
   *
   * @param meters The position goal to set in meters.
   */
  public default void setPivotGoal(Rotation2d rotation) {}

  /**
   * Sets the gains for the intake
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  public default void updateIntakeGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  /**
   * Sets the constraints for the intake.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  public default void updateIntakeConstraints(double maxAcceleration, double cruisingVelocity) {}
}
