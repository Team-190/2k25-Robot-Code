package frc.robot.subsystems.v1_gamma.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class V1_GammaClimberIOSim implements V1_GammaClimberIO {
  private final DCMotorSim sim;

  private double appliedVolts;

  public V1_GammaClimberIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaClimberConstants.MOTOR_CONFIG, 0.004, V1_GammaClimberConstants.GEAR_RATIO),
            V1_GammaClimberConstants.MOTOR_CONFIG);

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.position = Rotation2d.fromRadians(sim.getAngularPositionRad());
    inputs.velocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps =
        sim.getCurrentDrawAmps() / V1_GammaClimberConstants.GEARBOX_EFFICIENCY;
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public boolean isClimbed() {
    return sim.getAngularVelocityRadPerSec() == 0;
  }
}
