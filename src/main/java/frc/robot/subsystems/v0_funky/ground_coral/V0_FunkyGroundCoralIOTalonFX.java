package frc.robot.subsystems.v0_funky.ground_coral;

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

public class V0_FunkyGroundCoralIOTalonFX implements V0_FunkyGroundCoralIO {
  private final TalonFX groundCoralLeader;
  private final TalonFX groundCoralFollower;

  private final TalonFXConfiguration config;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temperatureCelcius;

  private final VoltageOut voltageRequest;

  public V0_FunkyGroundCoralIOTalonFX() {
    groundCoralLeader = new TalonFX(V0_FunkyGroundCoralConstants.GROUND_CORAL_CAN_ID);
    // talonFX = new TalonFX(V0_FunkyGroundCoralConstants.GROUND_CORAL_CAN_ID);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = V0_FunkyGroundCoralConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> groundCoralLeader.getConfigurator().apply(config, 0.25));

    positionRotations = groundCoralLeader.getPosition();
    velocityRotationsPerSecond = groundCoralLeader.getVelocity();
    appliedVoltage = groundCoralLeader.getMotorVoltage();
    supplyCurrentAmps = groundCoralLeader.getSupplyCurrent();
    torqueCurrentAmps = groundCoralLeader.getTorqueCurrent();
    temperatureCelcius = groundCoralLeader.getDeviceTemp();

    voltageRequest = new VoltageOut(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRotations,
        velocityRotationsPerSecond,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temperatureCelcius);

    groundCoralLeader.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GoundCoralIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionRotations,
        velocityRotationsPerSecond,
        appliedVoltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temperatureCelcius);

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelcius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    groundCoralLeader.setControl(voltageRequest.withOutput(volts));
  }
}
