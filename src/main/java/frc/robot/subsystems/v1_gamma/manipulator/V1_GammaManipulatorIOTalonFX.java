package frc.robot.subsystems.v1_gamma.manipulator;

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
import edu.wpi.first.wpilibj.DigitalInput;

public class V1_GammaManipulatorIOTalonFX implements V1_GammaManipulatorIO {
  private final TalonFX talonFX;

  private final TalonFXConfiguration config;
  private final DigitalInput sensor;

  private final StatusSignal<Angle> manipulatorPositionRotations;
  private final StatusSignal<AngularVelocity> manipulatorVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> manipulatorAppliedVoltage;
  private final StatusSignal<Current> manipulatorSupplyCurrentAmps;
  private final StatusSignal<Current> manipulatorTorqueCurrentAmps;
  private final StatusSignal<Temperature> manipulatorTemperatureCelsius;

  private final VoltageOut voltageRequest;

  public V1_GammaManipulatorIOTalonFX() {
    talonFX = new TalonFX(V1_GammaManipulatorConstants.MANIPULATOR_CAN_ID);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = V1_GammaManipulatorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    sensor = new DigitalInput(V1_GammaManipulatorConstants.CORAL_SENSOR_ID);

    manipulatorPositionRotations = talonFX.getPosition();
    manipulatorVelocityRotationsPerSecond = talonFX.getVelocity();
    manipulatorAppliedVoltage = talonFX.getMotorVoltage();
    manipulatorSupplyCurrentAmps = talonFX.getSupplyCurrent();
    manipulatorTorqueCurrentAmps = talonFX.getTorqueCurrent();
    manipulatorTemperatureCelsius = talonFX.getDeviceTemp();

    voltageRequest = new VoltageOut(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        manipulatorPositionRotations,
        manipulatorVelocityRotationsPerSecond,
        manipulatorAppliedVoltage,
        manipulatorSupplyCurrentAmps,
        manipulatorTorqueCurrentAmps,
        manipulatorTemperatureCelsius);

    talonFX.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        manipulatorPositionRotations,
        manipulatorVelocityRotationsPerSecond,
        manipulatorAppliedVoltage,
        manipulatorSupplyCurrentAmps,
        manipulatorTorqueCurrentAmps,
        manipulatorTemperatureCelsius);

    inputs.position = Rotation2d.fromRotations(manipulatorPositionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(manipulatorVelocityRotationsPerSecond.getValueAsDouble());
    inputs.appliedVolts = manipulatorAppliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = manipulatorSupplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = manipulatorTorqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = manipulatorTemperatureCelsius.getValueAsDouble();

    inputs.hasCoral =
        inputs.hasCoral
            ? sensor.get()
            : manipulatorTorqueCurrentAmps.getValueAsDouble()
                >= V1_GammaManipulatorConstants.MANIPULATOR_CURRENT_THRESHOLD;
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts));
  }
}
