package frc.robot.subsystems.v1_StackUp.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class V1_StackUpClimberIOSim implements V1_StackUpClimberIO {
  private final DCMotorSim sim;

  private double appliedVolts;

  public V1_StackUpClimberIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_StackUpClimberConstants.MOTOR_CONFIG,
                0.004,
                V1_StackUpClimberConstants.GEAR_RATIO),
            V1_StackUpClimberConstants.MOTOR_CONFIG);

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionRadians = sim.getAngularPositionRad();
    inputs.velocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps =
        sim.getCurrentDrawAmps() / V1_StackUpClimberConstants.GEARBOX_EFFICIENCY;
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public boolean isClimbed() {
    return false;
  }
}
