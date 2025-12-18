package frc.robot.subsystems.v3_Poot.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class V3_PootClimberIOSim implements V3_PootClimberIO {
  private final DCMotorSim sim;

  private double deploymentAppliedVolts;
  private double rollerAppliedVolts;

  public V3_PootClimberIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V3_PootClimberConstants.MOTOR_PARAMETERS.MOTOR_CONFIG(),
                0.004,
                V3_PootClimberConstants.MOTOR_PARAMETERS.GEAR_RATIO()),
            V3_PootClimberConstants.MOTOR_PARAMETERS.MOTOR_CONFIG());

    deploymentAppliedVolts = 0.0;
    rollerAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(V3_PootClimberIOInputs inputs) {
    deploymentAppliedVolts = MathUtil.clamp(deploymentAppliedVolts, -12.0, 12.0);
    sim.setInputVoltage(deploymentAppliedVolts);
    rollerAppliedVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);
    sim.setInputVoltage(rollerAppliedVolts);
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    // Update in standardized order
    inputs.deploymentPosition = Rotation2d.fromRadians(sim.getAngularPositionRad());
    inputs.deploymentVelocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.deploymentAppliedVolts = deploymentAppliedVolts;
    inputs.deploymentSupplyCurrentAmps =
        sim.getCurrentDrawAmps() / V3_PootClimberConstants.MOTOR_PARAMETERS.GEARBOX_EFFICIENCY();
    inputs.deploymentTorqueCurrentAmps = sim.getCurrentDrawAmps();

    inputs.rollerPosition = Rotation2d.fromRadians(sim.getAngularPositionRad());
    inputs.rollerVelocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerSupplyCurrentAmps =
        sim.getCurrentDrawAmps() / V3_PootClimberConstants.MOTOR_PARAMETERS.GEARBOX_EFFICIENCY();
    inputs.rollerTorqueCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setDeploymentVoltage(double volts) {
    deploymentAppliedVolts = volts;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerAppliedVolts = volts;
  }

  @Override
  public boolean isClimbed() {
    return false;
  }
}
