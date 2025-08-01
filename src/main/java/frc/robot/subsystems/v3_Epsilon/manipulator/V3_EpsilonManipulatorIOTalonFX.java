package frc.robot.subsystems.v3_Epsilon.manipulator;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
// import frc.robot.subsystems.v1_StackUp.manipulator.V3_EpsilonManipulatorConstants;

public class V3_EpsilonManipulatorIOTalonFX implements V3_EpsilonManipulatorIO {

  private final TalonFX pivotTalonFX;
  private final StatusSignal<Angle> pivotPositionRotations;
  private final StatusSignal<AngularVelocity> pivotVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> pivotAppliedVoltage;
  private final StatusSignal<Current> pivotSupplyCurrentAmps;
  private final StatusSignal<Current> pivotTorqueCurrentAmps;
  private final StatusSignal<Temperature> pivotTemperatureCelsius;
  private final StatusSignal<Double> pivotPositionSetpointRotations;
  private final StatusSignal<Double> pivotPositionErrorRotations;

  private final VoltageOut pivotVoltageRequest;
  private final MotionMagicVoltage pivotMotionMagicRequest;

  private final TalonFXConfiguration pivotConfig;

  private final TalonFX rollerTalonFX;
  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private final VoltageOut rollerVoltageRequest;
  private final TalonFXConfiguration rollerConfig;

  public V3_EpsilonManipulatorIOTalonFX() {
    pivotTalonFX = new TalonFX(V3_EpsilonManipulatorConstants.PIVOT_CAN_ID);
    rollerTalonFX = new TalonFX(V3_EpsilonManipulatorConstants.ROLLER_CAN_ID);

    pivotConfig = new TalonFXConfiguration();

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.SupplyCurrentLimit =
        V3_EpsilonManipulatorConstants.CURRENT_LIMITS.MANIPULATOR_SUPPLY_CURRENT_LIMIT();
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.Feedback.SensorToMechanismRatio =
        V3_EpsilonManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO();
    pivotConfig.Slot0 =
        Slot0Configs.from(V3_EpsilonManipulatorConstants.EMPTY_GAINS.toTalonFXSlotConfigs());
    pivotConfig.Slot1 =
        Slot1Configs.from(V3_EpsilonManipulatorConstants.CORAL_GAINS.toTalonFXSlotConfigs());
    pivotConfig.Slot2 =
        Slot2Configs.from(V3_EpsilonManipulatorConstants.ALGAE_GAINS.toTalonFXSlotConfigs());
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> pivotTalonFX.getConfigurator().apply(pivotConfig, 0.25));

    pivotPositionRotations = pivotTalonFX.getPosition();
    pivotVelocityRotationsPerSecond = pivotTalonFX.getVelocity();
    pivotAppliedVoltage = pivotTalonFX.getMotorVoltage();
    pivotSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
    pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
    pivotTemperatureCelsius = pivotTalonFX.getDeviceTemp();

    pivotPositionSetpointRotations = pivotTalonFX.getClosedLoopReference();
    pivotPositionErrorRotations = pivotTalonFX.getClosedLoopError();

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimit =
        V3_EpsilonManipulatorConstants.CURRENT_LIMITS.ROLLER_SUPPLY_CURRENT_LIMIT();
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> rollerTalonFX.getConfigurator().apply(rollerConfig, 0.25));

    rollerPositionRotations = rollerTalonFX.getPosition();
    rollerVelocityRotationsPerSecond = rollerTalonFX.getVelocity();
    rollerAppliedVoltage = rollerTalonFX.getMotorVoltage();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerTalonFX.getTorqueCurrent();
    rollerTemperatureCelsius = rollerTalonFX.getDeviceTemp();

    rollerVoltageRequest = new VoltageOut(0);
    pivotVoltageRequest = new VoltageOut(0);
    pivotMotionMagicRequest = new MotionMagicVoltage(0);

    registerSignals(
        false,
        pivotPositionRotations,
        pivotVelocityRotationsPerSecond,
        pivotAppliedVoltage,
        pivotSupplyCurrentAmps,
        pivotTorqueCurrentAmps,
        pivotTemperatureCelsius,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVoltage,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);

    pivotTalonFX.optimizeBusUtilization();
    rollerTalonFX.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {

    inputs.armPosition = new Rotation2d(pivotPositionRotations.getValue());
    inputs.armVelocityRadiansPerSecond =
        pivotVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);
    inputs.armAppliedVolts = pivotAppliedVoltage.getValueAsDouble();
    inputs.armSupplyCurrentAmps = pivotSupplyCurrentAmps.getValueAsDouble();
    inputs.armTorqueCurrentAmps = pivotTorqueCurrentAmps.getValueAsDouble();
    inputs.armTemperatureCelsius = pivotTemperatureCelsius.getValueAsDouble();

    inputs.rollerPosition = new Rotation2d(rollerPositionRotations.getValue());
    inputs.rollerVelocityRadiansPerSecond =
        Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();

    inputs.armPositionGoal = new Rotation2d(pivotMotionMagicRequest.getPositionMeasure());
    inputs.armPositionSetpoint =
        Rotation2d.fromRotations(pivotPositionSetpointRotations.getValueAsDouble());
    inputs.armPositionError =
        Rotation2d.fromRotations(pivotPositionErrorRotations.getValueAsDouble());
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotTalonFX.setControl(pivotVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerTalonFX.setControl(rollerVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPivotGoal(Rotation2d rotation) {
    pivotTalonFX.setControl(
        pivotMotionMagicRequest
            .withPosition(rotation.getRotations())
            .withEnableFOC(true));
  }

  @Override
  public void setSlot(int slot) {
    if (slot >= 0 && slot <= 2) {
      pivotTalonFX.setControl(pivotMotionMagicRequest.withSlot(slot));
    } else {
      throw new IllegalArgumentException("Invalid slot: " + slot);
    }
  }
}
