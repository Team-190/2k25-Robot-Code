package frc.robot.subsystems.v1_StackUp.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.shared.drive.TunerConstantsV1_StackUp;
import frc.robot.util.PhoenixUtil;

public class V1_StackUpClimberIOTalonFX implements V1_StackUpClimberIO {
  private final TalonFX talonFX;
  private final DigitalInput redundantSwitchOne;
  private final DigitalInput redundantSwitchTwo;

  private final TalonFXConfiguration config;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temperatureCelsius;

  private final VoltageOut voltageRequest;

  public V1_StackUpClimberIOTalonFX() {
    talonFX = new TalonFX(V1_StackUpClimberConstants.MOTOR_ID, TunerConstantsV1_StackUp.kCANBus);
    redundantSwitchOne = new DigitalInput(1);
    redundantSwitchTwo = new DigitalInput(2);

    config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit =
        V1_StackUpClimberConstants.CLIMBER_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit =
        V1_StackUpClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
    appliedVolts = talonFX.getMotorVoltage();
    supplyCurrentAmps = talonFX.getSupplyCurrent();
    torqueCurrentAmps = talonFX.getTorqueCurrent();
    temperatureCelsius = talonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temperatureCelsius);
    talonFX.optimizeBusUtilization();

    voltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temperatureCelsius);
    inputs.positionRadians = Units.rotationsToRadians(positionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();

    inputs.redundantSwitchOne = redundantSwitchOne.get();
    inputs.redundantSwitchTwo = redundantSwitchTwo.get();
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public boolean isClimbed() {
    return positionRotations.getValueAsDouble()
        >= Units.radiansToRotations(V1_StackUpClimberConstants.CLIMBER_CLIMBED_RADIANS);
  }
}
