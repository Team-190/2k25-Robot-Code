package frc.robot.subsystems.v3_Epsilon.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class V3_EpsilonClimberIOTalonFX implements V3_EpsilonClimberIO {

  private final TalonFX rollerTalonFX;
  private final TalonFX deploymentTalonFX;

  private final TalonFXConfiguration rollerConfig;
  private final TalonFXConfiguration deploymentConfig;

  private final VoltageOut rollerVoltageRequest;

  private final VoltageOut deploymentVoltageRequest;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private final StatusSignal<Angle> deploymentPositionRotations;
  private final StatusSignal<AngularVelocity> deploymentVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> deploymentAppliedVoltage;
  private final StatusSignal<Current> deploymentSupplyCurrentAmps;
  private final StatusSignal<Current> deploymentTorqueCurrentAmps;
  private final StatusSignal<Temperature> deploymentTemperatureCelsius;

  public V3_EpsilonClimberIOTalonFX() {
    rollerTalonFX = new TalonFX(V3_EpsilonClimberConstants.ROLLER_CAN_ID);
    deploymentTalonFX = new TalonFX(V3_EpsilonClimberConstants.DEPLOYMENT_CAN_ID);

    rollerConfig = new TalonFXConfiguration();
    rollerPositionRotations = rollerTalonFX.getPosition();
    rollerVelocityRotationsPerSecond = rollerTalonFX.getVelocity();
    rollerAppliedVoltage = rollerTalonFX.getSupplyVoltage();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerTalonFX.getTorqueCurrent();
    rollerTemperatureCelsius = rollerTalonFX.getDeviceTemp();
    tryUntilOk(5, () -> rollerTalonFX.getConfigurator().apply(rollerConfig, 0.25));

    deploymentConfig = new TalonFXConfiguration();
    deploymentPositionRotations = deploymentTalonFX.getPosition();
    deploymentVelocityRotationsPerSecond = deploymentTalonFX.getVelocity();
    deploymentAppliedVoltage = deploymentTalonFX.getSupplyVoltage();
    deploymentSupplyCurrentAmps = deploymentTalonFX.getSupplyCurrent();
    deploymentTorqueCurrentAmps = deploymentTalonFX.getTorqueCurrent();
    deploymentTemperatureCelsius = deploymentTalonFX.getDeviceTemp();
    tryUntilOk(5, () -> deploymentTalonFX.getConfigurator().apply(deploymentConfig, 0.25));

    deploymentVoltageRequest = new VoltageOut(0);
    rollerVoltageRequest = new VoltageOut(0);

    PhoenixUtil.registerSignals(
        false,
        deploymentPositionRotations,
        deploymentVelocityRotationsPerSecond,
        deploymentAppliedVoltage,
        deploymentSupplyCurrentAmps,
        deploymentTorqueCurrentAmps,
        deploymentTemperatureCelsius,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVoltage,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);
  }

  @Override
  public void updateInputs(V3_EpsilonClimberIOInputs inputs) {
    inputs.deploymentPosition = new Rotation2d(deploymentPositionRotations.getValue());
    inputs.deploymentVelocityRadiansPerSecond =
        deploymentVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);
    inputs.deploymentAppliedVolts = deploymentAppliedVoltage.getValueAsDouble();
    inputs.deploymentSupplyCurrentAmps = deploymentSupplyCurrentAmps.getValueAsDouble();
    inputs.deploymentTorqueCurrentAmps = deploymentTorqueCurrentAmps.getValueAsDouble();
    inputs.deploymentTemperatureCelsius = deploymentTemperatureCelsius.getValueAsDouble();

    inputs.rollerPosition = new Rotation2d(rollerPositionRotations.getValue());
    inputs.rollerVelocityRadiansPerSecond =
        rollerVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();
  }

  public void setdeploymentVoltage(double volts) {
    deploymentTalonFX.setControl(deploymentVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  public void setRollerVoltage(double volts) {
    rollerTalonFX.setControl(rollerVoltageRequest.withOutput(volts).withEnableFOC(true));
  }
}
