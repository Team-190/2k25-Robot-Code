package frc.robot.subsystems.shared.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim sim;

  private double appliedVolts;

  public ClimberIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ClimberConstants.MOTOR_PARAMETERS.MOTOR_CONFIG(),
                0.004,
                ClimberConstants.MOTOR_PARAMETERS.GEAR_RATIO()),
            ClimberConstants.MOTOR_PARAMETERS.MOTOR_CONFIG());

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    // Update in standardized order
    inputs.positionRadians = sim.getAngularPositionRad();
    inputs.velocityRadiansPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps =
        sim.getCurrentDrawAmps() / ClimberConstants.MOTOR_PARAMETERS.GEARBOX_EFFICIENCY();
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
