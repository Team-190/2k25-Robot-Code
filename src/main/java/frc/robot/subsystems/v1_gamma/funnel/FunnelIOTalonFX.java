package frc.robot.subsystems.v1_gamma.funnel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class FunnelIOTalonFX implements FunnelIO {
  private final TalonFX crabMotor;
  private final TalonFX intakeMotor;
  private final DigitalInput coralSensor;
  private final CANcoder crabEncoder;

  private final StatusSignal<Angle> crabPositionRotations;
  private final StatusSignal<AngularVelocity> crabVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> crabAppliedVolts;
  private final StatusSignal<Current> crabSupplyCurrentAmps;
  private final StatusSignal<Current> crabTorqueCurrentAmps;
  private final StatusSignal<Temperature> crabTemperatureCelsius;
  private final StatusSignal<Double> crabPositionSetpointRotations;
  private final StatusSignal<Double> crabPositionErrorRotations;

  private final StatusSignal<Angle> intakePositionRotations;
  private final StatusSignal<AngularVelocity> intakeVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeSupplyCurrentAmps;
  private final StatusSignal<Current> intakeTorqueCurrentAmps;
  private final StatusSignal<Temperature> intakeTemperatureCelsius;
  private final StatusSignal<Double> intakeVelocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> intakeVelocityErrorRotationsPerSecond;

  private final StatusSignal<Angle> encoderPositionRotations;

  private double crabGoalRadians;
  private double intakeGoalRadiansPerSecond;

  private VoltageOut voltageRequest;
  private NeutralOut neutralRequest;
  private MotionMagicVelocityVoltage velocityControlRequest;
  private MotionMagicVoltage positionControlRequest;

  private final Alert crabDisconnectedAlert =
      new Alert("Funnel Crab Motor Disconnected. Check CAN bus!", AlertType.ERROR);
  private final Alert intakeDisconnectedAlert =
      new Alert("Funnel Intake Motor Disconnected. Check CAN bus!", AlertType.ERROR);

  public FunnelIOTalonFX() {
    this.crabMotor = new TalonFX(FunnelConstants.CRAB_MOTOR_ID);
    this.intakeMotor = new TalonFX(FunnelConstants.INTAKE_MOTOR_ID);
    this.coralSensor = new DigitalInput(FunnelConstants.CORAL_SENSOR_ID);
    this.crabEncoder = new CANcoder(FunnelConstants.CRAB_MOTOR_ID);

    TalonFXConfiguration crabConfig = new TalonFXConfiguration();
    crabConfig.Feedback.FeedbackRemoteSensorID = crabEncoder.getDeviceID();
    crabConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    crabConfig.Feedback.SensorToMechanismRatio = FunnelConstants.CRAB_MOTOR_GEAR_RATIO;
    crabConfig.CurrentLimits.withSupplyCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.CRAB_SUPPLY_CURRENT_LIMIT());
    crabConfig.CurrentLimits.withStatorCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.CRAB_STATOR_CURRENT_LIMIT());
    crabConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    crabConfig.Slot0.kP = FunnelConstants.CRAB_MOTOR_GAINS.kP().get();
    crabConfig.Slot0.kD = FunnelConstants.CRAB_MOTOR_GAINS.kD().get();
    crabConfig.Slot0.kS = FunnelConstants.CRAB_MOTOR_GAINS.kS().get();
    crabConfig.Slot0.kV = FunnelConstants.CRAB_MOTOR_GAINS.kV().get();
    crabConfig.Slot0.kA = FunnelConstants.CRAB_MOTOR_GAINS.kA().get();
    crabConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        FunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS().get();
    crabConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        FunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS().get();
    crabConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.CRAB_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get();
    crabConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.CRAB_MOTOR_CONSTRAINTS.MAX_VELOCITY().get();

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.withSupplyCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.INTAKE_SUPPLY_CURRENT_LIMIT());
    intakeConfig.CurrentLimits.withStatorCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.INTAKE_STATOR_CURRENT_LIMIT());
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.Feedback.SensorToMechanismRatio = FunnelConstants.INTAKE_MOTOR_GEAR_RATIO;
    intakeConfig.Slot0.kP = FunnelConstants.INTAKE_MOTOR_GAINS.kP().get();
    intakeConfig.Slot0.kD = FunnelConstants.INTAKE_MOTOR_GAINS.kD().get();
    intakeConfig.Slot0.kS = FunnelConstants.INTAKE_MOTOR_GAINS.kS().get();
    intakeConfig.Slot0.kV = FunnelConstants.INTAKE_MOTOR_GAINS.kV().get();
    intakeConfig.Slot0.kA = FunnelConstants.INTAKE_MOTOR_GAINS.kA().get();
    intakeConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.INTAKE_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get();
    intakeConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.INTAKE_MOTOR_CONSTRAINTS.MAX_VELOCITY().get();

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset =
        FunnelConstants.CANCODER_ABSOLUTE_OFFSET_ROTATIONS.get();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    crabMotor.getConfigurator().apply(crabConfig);
    intakeMotor.getConfigurator().apply(intakeConfig);
    crabEncoder.getConfigurator().apply(canCoderConfig);

    crabPositionRotations = crabMotor.getPosition();
    crabVelocityRotationsPerSecond = crabMotor.getVelocity();
    crabAppliedVolts = crabMotor.getMotorVoltage();
    crabSupplyCurrentAmps = crabMotor.getSupplyCurrent();
    crabTorqueCurrentAmps = crabMotor.getTorqueCurrent();
    crabTemperatureCelsius = crabMotor.getDeviceTemp();
    crabPositionSetpointRotations = crabMotor.getClosedLoopReference();
    crabPositionErrorRotations = crabMotor.getClosedLoopError();

    intakePositionRotations = intakeMotor.getPosition();
    intakeVelocityRotationsPerSecond = intakeMotor.getVelocity();
    intakeAppliedVolts = intakeMotor.getMotorVoltage();
    intakeSupplyCurrentAmps = intakeMotor.getSupplyCurrent();
    intakeTorqueCurrentAmps = intakeMotor.getTorqueCurrent();
    intakeTemperatureCelsius = intakeMotor.getDeviceTemp();
    intakeVelocitySetpointRotationsPerSecond = intakeMotor.getClosedLoopReference();
    intakeVelocityErrorRotationsPerSecond = intakeMotor.getClosedLoopError();

    encoderPositionRotations = crabEncoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        crabPositionRotations,
        crabVelocityRotationsPerSecond,
        crabAppliedVolts,
        crabSupplyCurrentAmps,
        crabTorqueCurrentAmps,
        crabTemperatureCelsius,
        intakePositionRotations,
        intakeVelocityRotationsPerSecond,
        intakeAppliedVolts,
        intakeSupplyCurrentAmps,
        intakeTorqueCurrentAmps,
        intakeTemperatureCelsius);

    crabMotor.optimizeBusUtilization();
    intakeMotor.optimizeBusUtilization();

    voltageRequest = new VoltageOut(0.0);
    neutralRequest = new NeutralOut();
    velocityControlRequest = new MotionMagicVelocityVoltage(0.0);
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    boolean crabConnected =
        BaseStatusSignal.refreshAll(
                crabPositionRotations,
                crabVelocityRotationsPerSecond,
                crabAppliedVolts,
                crabSupplyCurrentAmps,
                crabTorqueCurrentAmps,
                crabTemperatureCelsius,
                crabPositionSetpointRotations,
                crabPositionErrorRotations)
            .isOK();

    boolean intakeConnected =
        BaseStatusSignal.refreshAll(
                intakePositionRotations,
                intakeVelocityRotationsPerSecond,
                intakeAppliedVolts,
                intakeSupplyCurrentAmps,
                intakeTorqueCurrentAmps,
                intakeTemperatureCelsius,
                intakeVelocitySetpointRotationsPerSecond,
                intakeVelocityErrorRotationsPerSecond)
            .isOK();

    crabDisconnectedAlert.set(!crabConnected);
    intakeDisconnectedAlert.set(!intakeConnected);

    encoderPositionRotations.refresh();

    inputs.crabPositionRadians = Units.rotationsToRadians(crabPositionRotations.getValueAsDouble());
    inputs.crabVelocityRadiansPerSecond =
        Units.rotationsToRadians(crabVelocityRotationsPerSecond.getValueAsDouble());
    inputs.crabGoalRadians = crabGoalRadians;
    inputs.crabAppliedVolts = crabAppliedVolts.getValueAsDouble();
    inputs.crabSupplyCurrentAmps = crabSupplyCurrentAmps.getValueAsDouble();
    inputs.crabTorqueCurrentAmps = crabTorqueCurrentAmps.getValueAsDouble();
    inputs.crabTemperatureCelsius = crabTemperatureCelsius.getValueAsDouble();
    inputs.crabPositionSetpointRadians = crabPositionSetpointRotations.getValueAsDouble();
    inputs.crabPositionErrorRadians = crabPositionErrorRotations.getValueAsDouble();
    inputs.encoderPositionRadians =
        Units.rotationsToRadians(encoderPositionRotations.getValueAsDouble());

    inputs.intakePositionRadians =
        Units.rotationsToRadians(intakePositionRotations.getValueAsDouble());
    inputs.intakeVelocityRadiansPerSecond =
        Units.rotationsToRadians(intakeVelocityRotationsPerSecond.getValueAsDouble());
    inputs.intakeGoalRadiansPerSecond = intakeGoalRadiansPerSecond;
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeSupplyCurrentAmps = intakeSupplyCurrentAmps.getValueAsDouble();
    inputs.intakeTorqueCurrentAmps = intakeTorqueCurrentAmps.getValueAsDouble();
    inputs.intakeTemperatureCelsius = intakeTemperatureCelsius.getValueAsDouble();
    inputs.intakeVelocitySetpointRadiansPerSecond =
        intakeVelocitySetpointRotationsPerSecond.getValueAsDouble();
    inputs.intakeVelocityErrorRadiansPerSecond =
        intakeVelocityErrorRotationsPerSecond.getValueAsDouble();

    inputs.hasCoral = coralSensor.get();
  }

  @Override
  public void setCrabVoltage(double volts) {
    crabMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setCrabPosition(double radians) {
    crabGoalRadians = radians;
    crabMotor.setControl(
        positionControlRequest.withPosition(Units.radiansToRotations(radians)).withEnableFOC(true));
  }

  @Override
  public void setIntakeVelocity(double radiansPerSecond) {
    intakeGoalRadiansPerSecond = radiansPerSecond;
    intakeMotor.setControl(
        velocityControlRequest
            .withVelocity(Units.radiansToRotations(radiansPerSecond))
            .withEnableFOC(true));
  }

  @Override
  public void stopIntake() {
    intakeMotor.setControl(neutralRequest);
  }

  @Override
  public boolean atCrabGoal() {
    return Math.abs(
            crabGoalRadians - Units.rotationsToRadians(crabPositionRotations.getValueAsDouble()))
        < FunnelConstants.CRAB_MOTOR_CONSTRAINTS.GOAL_TOLERANCE().get();
  }

  @Override
  public boolean atIntakeGoal() {
    return Math.abs(
            intakeGoalRadiansPerSecond
                - Units.rotationsToRadians(intakeVelocityRotationsPerSecond.getValueAsDouble()))
        < FunnelConstants.INTAKE_MOTOR_CONSTRAINTS.GOAL_TOLERANCE().get();
  }
}
