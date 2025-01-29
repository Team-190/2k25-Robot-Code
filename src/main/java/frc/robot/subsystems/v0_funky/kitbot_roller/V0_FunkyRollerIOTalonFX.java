package frc.robot.subsystems.v0_funky.kitbot_roller;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class V0_FunkyRollerIOTalonFX implements V0_FunkyRollerIO {
  private final TalonFX talonFX;

  private final TalonFXConfiguration config;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelcius;

  private final VoltageOut voltageRequest;

  public V0_FunkyRollerIOTalonFX() {
    talonFX = new TalonFX(V0_FunkyRollerConstants.ROLLER_CAN_ID);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = V0_FunkyRollerConstants.SupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    rollerPositionRotations = talonFX.getPosition();
    rollerVelocityRotationsPerSecond = talonFX.getVelocity();
    rollerAppliedVoltage = talonFX.getMotorVoltage();
    rollerSupplyCurrentAmps = talonFX.getSupplyCurrent();
    rollerTorqueCurrentAmps = talonFX.getTorqueCurrent();
    rollerTemperatureCelcius = talonFX.getDeviceTemp();

    voltageRequest = new VoltageOut(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVoltage,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelcius);

    talonFX.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVoltage,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelcius);

    inputs.position = Rotation2d.fromRotations(rollerPositionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.appliedVolts = rollerAppliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = rollerTemperatureCelcius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts));
  }
}
