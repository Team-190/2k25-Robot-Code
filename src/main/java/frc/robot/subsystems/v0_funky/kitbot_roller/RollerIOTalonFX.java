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

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX roller;

  private final TalonFXConfiguration rollerConfig;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelcius;

  private final VoltageOut voltageRequest;

  public RollerIOTalonFX() {
    roller = new TalonFX(RollerConstants.ROLLER_CAN_ID, "rio");

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = RollerConstants.SupplyCurrentLimit;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig, 0.25));

    rollerPositionRotations = roller.getPosition();
    rollerVelocityRotationsPerSecond = roller.getVelocity();
    rollerAppliedVoltage = roller.getMotorVoltage();
    rollerSupplyCurrentAmps = roller.getSupplyCurrent();
    rollerTorqueCurrentAmps = roller.getTorqueCurrent();
    rollerTemperatureCelcius = roller.getDeviceTemp();

    voltageRequest = new VoltageOut(0);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVoltage,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelcius);

    roller.optimizeBusUtilization();
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
    roller.setControl(voltageRequest.withOutput(volts));
  }
}
