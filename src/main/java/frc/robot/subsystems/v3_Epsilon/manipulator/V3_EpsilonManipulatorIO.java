package frc.robot.subsystems.v3_Epsilon.manipulator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface V3_EpsilonManipulatorIO {

  @AutoLog
  public static class ManipulatorIOInputs {
    public Rotation2d armPosition = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    
    
    public Rotation2d armPositionGoal = new Rotation2d();
    public Rotation2d armPositionSetpoint = new Rotation2d();
    public Rotation2d armPositionError = new Rotation2d();
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
  public default void updateInputs(ManipulatorIOInputs inputs) {}

  /**
   * Sets the voltage for the manipulator.
   *
   * @param volts The voltage to set.
   */
  public default void setVoltage(double volts) {}
}
