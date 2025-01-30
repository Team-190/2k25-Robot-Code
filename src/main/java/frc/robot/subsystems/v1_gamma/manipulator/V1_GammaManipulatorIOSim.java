package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class V1_GammaManipulatorIOSim implements V1_GammaManipulatorIO {
  private final DCMotorSim sim;

  private double appliedVolts;

  public V1_GammaManipulatorIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 3.0),
            DCMotor.getKrakenX60Foc(1));

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.position = Rotation2d.fromRadians(sim.getAngularPositionRad());
    inputs.velocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
