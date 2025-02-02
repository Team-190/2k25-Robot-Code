package frc.robot.subsystems.v1_gamma.funnel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class V1_GammaFunnelIOTalonFX implements V1_GammaFunnelIO {
  private final TalonFX serializerTalonFX;
  private final TalonFX rollerTalonFX;
  private final DigitalInput coralSensor;
  private final CANcoder serializerCANcoder;

  private final TalonFXConfiguration serializerConfig;
  private final TalonFXConfiguration rollerConfig;
  private final CANcoderConfiguration cancoderConfig;

  private final StatusSignal<Angle> serializerPositionRotations;
  private final StatusSignal<AngularVelocity> serializerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> serializerAppliedVolts;
  private final StatusSignal<Current> serializerSupplyCurrentAmps;
  private final StatusSignal<Current> serializerTorqueCurrentAmps;
  private final StatusSignal<Temperature> serializerTemperatureCelsius;
  private final StatusSignal<Double> serializerPositionSetpointRotations;
  private final StatusSignal<Double> serializerPositionErrorRotations;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private final StatusSignal<Angle> cancoderPositionRotations;

  private Rotation2d serializerGoal;

  private MotionMagicVoltage positionControlRequest;
  private VoltageOut voltageRequest;
  private NeutralOut neutralRequest;

  public V1_GammaFunnelIOTalonFX() {
    this.serializerTalonFX = new TalonFX(V1_GammaFunnelConstants.SERIALIZER_MOTOR_ID);
    this.rollerTalonFX = new TalonFX(V1_GammaFunnelConstants.ROLLER_MOTOR_ID);
    this.coralSensor = new DigitalInput(V1_GammaFunnelConstants.CORAL_SENSOR_ID);
    this.serializerCANcoder = new CANcoder(V1_GammaFunnelConstants.SERIALIZER_MOTOR_ID);

    serializerConfig = new TalonFXConfiguration();
    serializerConfig.Feedback.FeedbackRemoteSensorID = serializerCANcoder.getDeviceID();
    serializerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    serializerConfig.Feedback.SensorToMechanismRatio =
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_GEAR_RATIO;
    serializerConfig.Feedback.RotorToSensorRatio =
        V1_GammaFunnelConstants.SERIALIZER_CANCODER_GEAR_RATIO;
    serializerConfig.CurrentLimits.withSupplyCurrentLimit(
        V1_GammaFunnelConstants.CURRENT_LIMITS.SERIALIZER_SUPPLY_CURRENT_LIMIT());
    serializerConfig.CurrentLimits.withStatorCurrentLimit(
        V1_GammaFunnelConstants.CURRENT_LIMITS.SERIALIZER_STATOR_CURRENT_LIMIT());
    serializerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    serializerConfig.Slot0.kP = V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kP().get();
    serializerConfig.Slot0.kD = V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kD().get();
    serializerConfig.Slot0.kS = V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kS().get();
    serializerConfig.Slot0.kV = V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kV().get();
    serializerConfig.Slot0.kA = V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kA().get();
    serializerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V1_GammaFunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS();
    serializerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V1_GammaFunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS();
    serializerConfig.MotionMagic.MotionMagicAcceleration =
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get();
    serializerConfig.MotionMagic.MotionMagicCruiseVelocity =
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get();

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.withSupplyCurrentLimit(
        V1_GammaFunnelConstants.CURRENT_LIMITS.ROLLER_SUPPLY_CURRENT_LIMIT());
    rollerConfig.CurrentLimits.withStatorCurrentLimit(
        V1_GammaFunnelConstants.CURRENT_LIMITS.ROLLER_STATOR_CURRENT_LIMIT());
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Feedback.SensorToMechanismRatio = V1_GammaFunnelConstants.ROLLER_MOTOR_GEAR_RATIO;

    cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset =
        V1_GammaFunnelConstants.CANCODER_ABSOLUTE_OFFSET_RADIANS.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    serializerTalonFX.getConfigurator().apply(serializerConfig);
    rollerTalonFX.getConfigurator().apply(rollerConfig);
    serializerCANcoder.getConfigurator().apply(cancoderConfig);

    serializerPositionRotations = serializerTalonFX.getPosition();
    serializerVelocityRotationsPerSecond = serializerTalonFX.getVelocity();
    serializerAppliedVolts = serializerTalonFX.getMotorVoltage();
    serializerSupplyCurrentAmps = serializerTalonFX.getSupplyCurrent();
    serializerTorqueCurrentAmps = serializerTalonFX.getTorqueCurrent();
    serializerTemperatureCelsius = serializerTalonFX.getDeviceTemp();
    serializerPositionSetpointRotations = serializerTalonFX.getClosedLoopReference();
    serializerPositionErrorRotations = serializerTalonFX.getClosedLoopError();

    rollerPositionRotations = rollerTalonFX.getPosition();
    rollerVelocityRotationsPerSecond = rollerTalonFX.getVelocity();
    rollerAppliedVolts = rollerTalonFX.getMotorVoltage();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerTalonFX.getTorqueCurrent();
    rollerTemperatureCelsius = rollerTalonFX.getDeviceTemp();

    cancoderPositionRotations = serializerCANcoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        serializerPositionRotations,
        serializerVelocityRotationsPerSecond,
        serializerAppliedVolts,
        serializerSupplyCurrentAmps,
        serializerTorqueCurrentAmps,
        serializerTemperatureCelsius,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);

    serializerTalonFX.optimizeBusUtilization();
    rollerTalonFX.optimizeBusUtilization();

    voltageRequest = new VoltageOut(0.0);
    neutralRequest = new NeutralOut();
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        serializerPositionRotations,
        serializerVelocityRotationsPerSecond,
        serializerAppliedVolts,
        serializerSupplyCurrentAmps,
        serializerTorqueCurrentAmps,
        serializerTemperatureCelsius,
        serializerPositionSetpointRotations,
        serializerPositionErrorRotations);

    BaseStatusSignal.refreshAll(
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);

    cancoderPositionRotations.refresh();

    inputs.serializerPosition =
        Rotation2d.fromRotations(serializerPositionRotations.getValueAsDouble());
    inputs.serializerAbsolutePosition =
        Rotation2d.fromRotations(cancoderPositionRotations.getValueAsDouble());
    inputs.serializerVelocityRadiansPerSecond =
        Units.rotationsToRadians(serializerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.serializerAppliedVolts = serializerAppliedVolts.getValueAsDouble();
    inputs.serializerSupplyCurrentAmps = serializerSupplyCurrentAmps.getValueAsDouble();
    inputs.serializerTorqueCurrentAmps = serializerTorqueCurrentAmps.getValueAsDouble();
    inputs.serializerTemperatureCelsius = serializerTemperatureCelsius.getValueAsDouble();
    inputs.serializerGoal = serializerGoal;
    inputs.serializerPositionSetpoint =
        Rotation2d.fromRotations(serializerPositionSetpointRotations.getValueAsDouble());
    inputs.serializerPositionError =
        Rotation2d.fromRotations(serializerPositionErrorRotations.getValueAsDouble());

    inputs.rollerPosition = Rotation2d.fromRotations(rollerPositionRotations.getValueAsDouble());
    inputs.rollerVelocityRadiansPerSecond =
        Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();

    inputs.hasCoral = coralSensor.get();
  }

  @Override
  public void setSerializerVoltage(double volts) {
    serializerTalonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerTalonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setSerializerPosition(Rotation2d position) {
    serializerGoal = position;
    serializerTalonFX.setControl(
        positionControlRequest.withPosition(position.getRotations()).withEnableFOC(true));
  }

  @Override
  public void stopRoller() {
    rollerTalonFX.setControl(neutralRequest);
  }

  @Override
  public boolean atSerializerGoal() {
    return Math.abs(
            serializerGoal.getRadians()
                - Units.rotationsToRadians(serializerPositionRotations.getValueAsDouble()))
        < V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.GOAL_TOLERANCE().get();
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    TalonFXConfiguration serializerConfig = new TalonFXConfiguration();
    serializerConfig.Slot0.kP = kP;
    serializerConfig.Slot0.kD = kD;
    serializerConfig.Slot0.kS = kS;
    serializerConfig.Slot0.kV = kV;
    serializerConfig.Slot0.kA = kA;
    serializerTalonFX.getConfigurator().apply(serializerConfig);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    TalonFXConfiguration serializerConfig = new TalonFXConfiguration();
    serializerConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    serializerConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    serializerTalonFX.getConfigurator().apply(serializerConfig);
  }
}
