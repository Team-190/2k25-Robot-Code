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
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class FunnelIOTalonFX implements FunnelIO {
  private final TalonFX serializerMotor;
  private final TalonFX rollerMotor;
  private final DigitalInput coralSensor;
  private final CANcoder serializerEncoder;

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

  private final StatusSignal<Angle> encoderPositionRotations;

  private double serializerGoalRadians;

  private VoltageOut voltageRequest;
  private NeutralOut neutralRequest;
  private MotionMagicVoltage positionControlRequest;

  private final Alert serializerDisconnectedAlert =
      new Alert("Funnel Serializer Motor Disconnected. Check CAN bus!", AlertType.ERROR);
  private final Alert rollerDisconnectedAlert =
      new Alert("Funnel Roller Motor Disconnected. Check CAN bus!", AlertType.ERROR);

  public FunnelIOTalonFX() {
    this.serializerMotor = new TalonFX(FunnelConstants.SERIALIZER_MOTOR_ID);
    this.rollerMotor = new TalonFX(FunnelConstants.ROLLER_MOTOR_ID);
    this.coralSensor = new DigitalInput(FunnelConstants.CORAL_SENSOR_ID);
    this.serializerEncoder = new CANcoder(FunnelConstants.SERIALIZER_MOTOR_ID);

    TalonFXConfiguration serializerConfig = new TalonFXConfiguration();
    serializerConfig.Feedback.FeedbackRemoteSensorID = serializerEncoder.getDeviceID();
    serializerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    serializerConfig.Feedback.SensorToMechanismRatio = FunnelConstants.SERIALIZER_MOTOR_GEAR_RATIO;
    serializerConfig.CurrentLimits.withSupplyCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.SERIALIZER_SUPPLY_CURRENT_LIMIT());
    serializerConfig.CurrentLimits.withStatorCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.SERIALIZER_STATOR_CURRENT_LIMIT());
    serializerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    serializerConfig.Slot0.kP = FunnelConstants.SERIALIZER_MOTOR_GAINS.kP().get();
    serializerConfig.Slot0.kD = FunnelConstants.SERIALIZER_MOTOR_GAINS.kD().get();
    serializerConfig.Slot0.kS = FunnelConstants.SERIALIZER_MOTOR_GAINS.kS().get();
    serializerConfig.Slot0.kV = FunnelConstants.SERIALIZER_MOTOR_GAINS.kV().get();
    serializerConfig.Slot0.kA = FunnelConstants.SERIALIZER_MOTOR_GAINS.kA().get();
    serializerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        FunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS().get();
    serializerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        FunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS().get();
    serializerConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get();
    serializerConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get();

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.withSupplyCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.ROLLER_SUPPLY_CURRENT_LIMIT());
    rollerConfig.CurrentLimits.withStatorCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.ROLLER_STATOR_CURRENT_LIMIT());
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Feedback.SensorToMechanismRatio = FunnelConstants.ROLLER_MOTOR_GEAR_RATIO;

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset =
        Units.radiansToRotations(FunnelConstants.CANCODER_ABSOLUTE_OFFSET_RADIANS.get());
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    serializerMotor.getConfigurator().apply(serializerConfig);
    rollerMotor.getConfigurator().apply(rollerConfig);
    serializerEncoder.getConfigurator().apply(canCoderConfig);

    serializerPositionRotations = serializerMotor.getPosition();
    serializerVelocityRotationsPerSecond = serializerMotor.getVelocity();
    serializerAppliedVolts = serializerMotor.getMotorVoltage();
    serializerSupplyCurrentAmps = serializerMotor.getSupplyCurrent();
    serializerTorqueCurrentAmps = serializerMotor.getTorqueCurrent();
    serializerTemperatureCelsius = serializerMotor.getDeviceTemp();
    serializerPositionSetpointRotations = serializerMotor.getClosedLoopReference();
    serializerPositionErrorRotations = serializerMotor.getClosedLoopError();

    rollerPositionRotations = rollerMotor.getPosition();
    rollerVelocityRotationsPerSecond = rollerMotor.getVelocity();
    rollerAppliedVolts = rollerMotor.getMotorVoltage();
    rollerSupplyCurrentAmps = rollerMotor.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerMotor.getTorqueCurrent();
    rollerTemperatureCelsius = rollerMotor.getDeviceTemp();

    encoderPositionRotations = serializerEncoder.getPosition();

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

    serializerMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();

    voltageRequest = new VoltageOut(0.0);
    neutralRequest = new NeutralOut();
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    boolean serializerConnected =
        BaseStatusSignal.refreshAll(
                serializerPositionRotations,
                serializerVelocityRotationsPerSecond,
                serializerAppliedVolts,
                serializerSupplyCurrentAmps,
                serializerTorqueCurrentAmps,
                serializerTemperatureCelsius,
                serializerPositionSetpointRotations,
                serializerPositionErrorRotations)
            .isOK();

    boolean rollerConnected =
        BaseStatusSignal.refreshAll(
                rollerPositionRotations,
                rollerVelocityRotationsPerSecond,
                rollerAppliedVolts,
                rollerSupplyCurrentAmps,
                rollerTorqueCurrentAmps,
                rollerTemperatureCelsius)
            .isOK();

    serializerDisconnectedAlert.set(!serializerConnected);
    rollerDisconnectedAlert.set(!rollerConnected);

    encoderPositionRotations.refresh();

    inputs.serializerPosition =
        Rotation2d.fromRotations(serializerPositionRotations.getValueAsDouble());
        inputs.serializerAbsolutePosition =
        Rotation2d.fromRotations(encoderPositionRotations.getValueAsDouble());
    inputs.serializerVelocityRadiansPerSecond =
        Units.rotationsToRadians(serializerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.serializerAppliedVolts = serializerAppliedVolts.getValueAsDouble();
    inputs.serializerSupplyCurrentAmps = serializerSupplyCurrentAmps.getValueAsDouble();
    inputs.serializerTorqueCurrentAmps = serializerTorqueCurrentAmps.getValueAsDouble();
    inputs.serializerTemperatureCelsius = serializerTemperatureCelsius.getValueAsDouble();
    inputs.serializerGoal = Rotation2d.fromRadians(serializerGoalRadians);
    inputs.serializerPositionSetpoint =
    Rotation2d.fromRotations(serializerPositionSetpointRotations.getValueAsDouble());
    inputs.serializerPositionError = Rotation2d.fromRotations(serializerPositionErrorRotations.getValueAsDouble());

    inputs.rollerPosition =
    Rotation2d.fromRotations(rollerPositionRotations.getValueAsDouble());
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
    serializerMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setSerializerPosition(double radians) {
    serializerGoalRadians = radians;
    serializerMotor.setControl(
        positionControlRequest.withPosition(Units.radiansToRotations(radians)).withEnableFOC(true));
  }

  @Override
  public void stopRoller() {
    rollerMotor.setControl(neutralRequest);
  }

  @Override
  public boolean atSerializerGoal() {
    return Math.abs(
            serializerGoalRadians
                - Units.rotationsToRadians(serializerPositionRotations.getValueAsDouble()))
        < FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.GOAL_TOLERANCE().get();
  }
}
