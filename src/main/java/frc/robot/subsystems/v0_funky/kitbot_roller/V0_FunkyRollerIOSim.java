package frc.robot.subsystems.v0_funky.kitbot_roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class V0_FunkyRollerIOSim implements V0_FunkyRollerIO {
  private final DCMotorSim sim;

  private double appliedVolts;

  public V0_FunkyRollerIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 3.0),
            DCMotor.getKrakenX60Foc(1));

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
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
