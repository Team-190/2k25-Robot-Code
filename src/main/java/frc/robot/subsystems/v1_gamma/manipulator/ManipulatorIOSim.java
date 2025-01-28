package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ManipulatorIOSim implements ManipulatorIO {
  private final DCMotorSim manipulatorSim;

  private double appliedVolts;

  public ManipulatorIOSim() {
    manipulatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 3.0),
            DCMotor.getKrakenX60Foc(1));

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    manipulatorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    manipulatorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.position = Rotation2d.fromRadians(manipulatorSim.getAngularPositionRad());
    inputs.velocityRadiansPerSecond = manipulatorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = manipulatorSim.getCurrentDrawAmps();
  }

  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
