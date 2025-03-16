package frc.robot.subsystems.v2_Redundancy.inkake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.shared.drive.TunerConstantsV2_Redundancy;

public class V2_RedundancyIntakeIOTalonFX implements V2_RedundancyIntakeIO {
  private final TalonFX extensionTalonFX;
  private final TalonFX rollerTalonFX;

  private final TalonFXConfiguration extensionConfig;
  private final TalonFXConfiguration rollerConfig;

  private final StatusSignal<Angle> extensionPositionRotations;
  private final StatusSignal<AngularVelocity> extensionVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> extensionAppliedVolts;
  private final StatusSignal<Current> extensionSupplyCurrentAmps;
  private final StatusSignal<Current> extensionTorqueCurrentAmps;
  private final StatusSignal<Temperature> extensionTemperatureCelsius;
  private final StatusSignal<Double> extensionPositionSetpointRotations;
  private final StatusSignal<Double> extensionPositionErrorRotations;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private double extensionGoal;

  private MotionMagicVoltage positionControlRequest;
  private VoltageOut voltageRequest;
  private NeutralOut neutralRequest;

  public V2_RedundancyIntakeIOTalonFX() {
    extensionTalonFX =
        new TalonFX(
            V2_RedundancyIntakeConstants.EXTENSION_MOTOR_ID, TunerConstantsV2_Redundancy.kCANBus);
    rollerTalonFX = new TalonFX(V2_RedundancyIntakeConstants.ROLLER_MOTOR_ID);

    extensionConfig = new TalonFXConfiguration();
    extensionConfig.Feedback.SensorToMechanismRatio =
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GEAR_RATIO;
    extensionConfig.CurrentLimits.withSupplyCurrentLimit(
        V2_RedundancyIntakeConstants.CURRENT_LIMITS.EXTENSION_SUPPLY_CURRENT_LIMIT());
    extensionConfig.CurrentLimits.withStatorCurrentLimit(
        V2_RedundancyIntakeConstants.CURRENT_LIMITS.EXTENSION_STATOR_CURRENT_LIMIT());
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    extensionConfig.Slot0.kP = V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kP().get();
    extensionConfig.Slot0.kD = V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kD().get();
    extensionConfig.Slot0.kS = V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kS().get();
    extensionConfig.Slot0.kV = V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kV().get();
    extensionConfig.Slot0.kA = V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kA().get();
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(V2_RedundancyIntakeConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS());
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(V2_RedundancyIntakeConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS());
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    extensionConfig.MotionMagic.MotionMagicAcceleration =
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get();
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity =
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_VELOCITY().get();

    tryUntilOk(5, () -> extensionTalonFX.getConfigurator().apply(extensionConfig, 0.25));

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.withSupplyCurrentLimit(
        V2_RedundancyIntakeConstants.CURRENT_LIMITS.ROLLER_SUPPLY_CURRENT_LIMIT());
    rollerConfig.CurrentLimits.withStatorCurrentLimit(
        V2_RedundancyIntakeConstants.CURRENT_LIMITS.ROLLER_STATOR_CURRENT_LIMIT());
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Feedback.SensorToMechanismRatio =
        V2_RedundancyIntakeConstants.ROLLER_MOTOR_GEAR_RATIO;

    tryUntilOk(5, () -> rollerTalonFX.getConfigurator().apply(rollerConfig, 0.25));

    extensionPositionRotations = extensionTalonFX.getPosition();
    extensionVelocityRotationsPerSecond = extensionTalonFX.getVelocity();
    extensionAppliedVolts = extensionTalonFX.getMotorVoltage();
    extensionSupplyCurrentAmps = extensionTalonFX.getSupplyCurrent();
    extensionTorqueCurrentAmps = extensionTalonFX.getTorqueCurrent();
    extensionTemperatureCelsius = extensionTalonFX.getDeviceTemp();
    extensionPositionSetpointRotations = extensionTalonFX.getClosedLoopReference();
    extensionPositionErrorRotations = extensionTalonFX.getClosedLoopError();

    rollerPositionRotations = rollerTalonFX.getPosition();
    rollerVelocityRotationsPerSecond = rollerTalonFX.getVelocity();
    rollerAppliedVolts = rollerTalonFX.getMotorVoltage();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerTalonFX.getTorqueCurrent();
    rollerTemperatureCelsius = rollerTalonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        extensionPositionRotations,
        extensionVelocityRotationsPerSecond,
        extensionAppliedVolts,
        extensionSupplyCurrentAmps,
        extensionTorqueCurrentAmps,
        extensionTemperatureCelsius,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);

    extensionTalonFX.optimizeBusUtilization();
    rollerTalonFX.optimizeBusUtilization();

    voltageRequest = new VoltageOut(0.0);
    neutralRequest = new NeutralOut();
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        extensionPositionRotations,
        extensionVelocityRotationsPerSecond,
        extensionAppliedVolts,
        extensionSupplyCurrentAmps,
        extensionTorqueCurrentAmps,
        extensionTemperatureCelsius,
        extensionPositionSetpointRotations,
        extensionPositionErrorRotations);

    BaseStatusSignal.refreshAll(
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);

    inputs.extensionPositionMeters =
        (extensionPositionRotations.getValueAsDouble()
            * V2_RedundancyIntakeConstants.EXTENSION_MOTOR_METERS_PER_REV);
    inputs.extensionVelocityMetersPerSecond =
        Units.rotationsToRadians(
            extensionVelocityRotationsPerSecond.getValueAsDouble()
                * V2_RedundancyIntakeConstants.EXTENSION_MOTOR_METERS_PER_REV);
    inputs.extensionAppliedVolts = extensionAppliedVolts.getValueAsDouble();
    inputs.extensionSupplyCurrentAmps = extensionSupplyCurrentAmps.getValueAsDouble();
    inputs.extensionTorqueCurrentAmps = extensionTorqueCurrentAmps.getValueAsDouble();
    inputs.extensionTemperatureCelsius = extensionTemperatureCelsius.getValueAsDouble();
    inputs.extensionGoal = extensionGoal;
    inputs.extensionPositionSetpoint = (extensionPositionSetpointRotations.getValueAsDouble());
    inputs.extensionPositionError = (extensionPositionErrorRotations.getValueAsDouble());

    inputs.rollerPosition = Rotation2d.fromRotations(rollerPositionRotations.getValueAsDouble());
    inputs.rollerVelocityRadiansPerSecond =
        Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setExtensionVoltage(double volts) {
    extensionTalonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerTalonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void stopRoller() {
    rollerTalonFX.setControl(neutralRequest);
  }

  @Override
  public void setExtensionGoal(double position) {
    extensionGoal = position;
    extensionTalonFX.setControl(
        positionControlRequest
            .withPosition(position / V2_RedundancyIntakeConstants.EXTENSION_MOTOR_METERS_PER_REV)
            .withEnableFOC(true));
  }

  @Override
  public boolean atExtensionPositionGoal() {
    return Math.abs(
            extensionGoal
                - (extensionPositionRotations.getValueAsDouble()
                    * V2_RedundancyIntakeConstants.EXTENSION_MOTOR_METERS_PER_REV))
        < V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.GOAL_TOLERANCE().get();
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    extensionConfig.Slot0.kP = kP;
    extensionConfig.Slot0.kD = kD;
    extensionConfig.Slot0.kS = kS;
    extensionConfig.Slot0.kV = kV;
    extensionConfig.Slot0.kA = kA;
    tryUntilOk(5, () -> extensionTalonFX.getConfigurator().apply(extensionConfig, 0.25));
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    extensionConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    tryUntilOk(5, () -> extensionTalonFX.getConfigurator().apply(extensionConfig, 0.25));
  }
}
