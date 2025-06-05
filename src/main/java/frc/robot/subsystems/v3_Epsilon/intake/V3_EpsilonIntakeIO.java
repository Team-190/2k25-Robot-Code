package frc.robot.subsystems.v3_Epsilon.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface V3_EpsilonIntakeIO{
    @AutoLog
    public static class IntakeIOInputs{
        public Rotation2d intakePosition= new Rotation2d();
        public double intakeVelocityRadiansPerSecond=0.0;
        public double intakeAppliedVolts=0.0;
        public double intakeSupplyCurrentAmps=0.0;
        public double intakeTorqueCurrentAmps=0.0;
        public double intakeTemperatureCelsius=0.0;
        public Rotation2d intakePositionGoal = new Rotation2d();
        public Rotation2d intakePositionSetpoint = new Rotation2d();
        public Rotation2d intakePositionError = new Rotation2d();

        public Rotation2d rollerPosition = new Rotation2d();
        public double rollerVelocityRadiansPerSecond = 0.0;
        public double rollerAccelerationRadiansPerSecondSquared = 0.0;
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
  public default void updateInputs(IntakeIOInputs inputs) {}

  /**
   * Sets the voltage for the intake.
   *
   * @param volts The voltage to set.
   */
  public default void setIntakeVoltage(double volts) {}

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
  public default void setIntakePositionGoal(Rotation2d rotatoion) {}

  /**
   * Sets the gains for the intake
   *
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  public default void updateSlot0IntakeGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  public default void updateSlot1IntakeGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {}

  /**
   * Sets the constraints for the intake.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  
}