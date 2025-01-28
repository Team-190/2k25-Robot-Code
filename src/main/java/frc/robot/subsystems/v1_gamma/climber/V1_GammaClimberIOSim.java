package frc.robot.subsystems.v1_gamma.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class V1_GammaClimberIOSim implements V1_GammaClimberIO {
  private final DCMotorSim motorSim;

  private double appliedVolts;

  public V1_GammaClimberIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                V1_GammaClimberConstants.MOTOR_CONFIG, 0.004, V1_GammaClimberConstants.GEAR_RATIO),
            V1_GammaClimberConstants.MOTOR_CONFIG);

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionRotations = motorSim.getAngularPositionRotations();
    inputs.velocityRotationsPerSecond =
        Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        V1_GammaClimberConstants.MOTOR_CONFIG.getCurrent(motorSim.getCurrentDrawAmps())
            * V1_GammaClimberConstants.GEARBOX_EFFICIENCY;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
  }

    @Override
    public void setCurrent(double amps) {
        appliedVolts = MathUtil.clamp(-12,((V1_GammaClimberConstants.MOTOR_CONFIG.KtNMPerAmp*motorSim.getAngularVelocityRadPerSec())/V1_GammaClimberConstants.GEARBOX_EFFICIENCY)+(amps * V1_GammaClimberConstants.MOTOR_CONFIG.rOhms),12);
    }

    @Override
    public boolean isClimbed() {
        return motorSim.getAngularVelocityRadPerSec() == 0;
    }
}
