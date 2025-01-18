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

public class ManipulatorIOTalonFX implements ManipulatorIO {
  private final TalonFX manipulator;

  private final TalonFXConfiguration manipulatorConfig;

  private final StatusSignal<Angle> manipulatorPositionRotations;
  private final StatusSignal<AngularVelocity> manipulatorVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> manipulatorAppliedVoltage;
  private final StatusSignal<Current> manipulatorSupplyCurrentAmps;
  private final StatusSignal<Current> manipulatorTorqueCurrentAmps;
  private final StatusSignal<Temperature> manipulatorTemperatureCelsius;

  private final VoltageOut voltageRequest;

  public ManipulatorIOTalonFX() {
    manipulator = new TalonFX(ManipulatorConstants.MANIPULATOR_CAN_ID);

    manipulatorConfig = new TalonFXConfiguration();
    manipulatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    manipulatorConfig.CurrentLimits.SupplyCurrentLimit = ManipulatorConstants.SupplyCurrentLimit;
    manipulatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> manipulator.getConfigurator().apply(manipulatorConfig, 0.25));

    manipulatorPositionRotations = manipulator.getPosition();
    manipulatorVelocityRotationsPerSecond = manipulator.getVelocity();
    manipulatorAppliedVoltage = manipulator.getMotorVoltage();
    manipulatorSupplyCurrentAmps = manipulator.getSupplyCurrent();
    manipulatorTorqueCurrentAmps = manipulator.getTorqueCurrent();
    manipulatorTemperatureCelsius = manipulator.getDeviceTemp();

    voltageRequest = new VoltageOut(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        manipulatorPositionRotations,
        manipulatorVelocityRotationsPerSecond,
        manipulatorAppliedVoltage,
        manipulatorSupplyCurrentAmps,
        manipulatorTorqueCurrentAmps,
        manipulatorTemperatureCelsius);

    manipulator.optimizeBusUtilization();
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
  }

  @Override
  public void setVoltage(double volts) {
    manipulator.setControl(voltageRequest.withOutput(volts));
  }
}
