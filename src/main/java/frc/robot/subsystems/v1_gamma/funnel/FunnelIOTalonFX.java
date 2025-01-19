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
  private final TalonFX clapperMotor;
  private final TalonFX rollerMotor;
  private final DigitalInput coralSensor;
  private final CANcoder clapperEncoder;

  private final StatusSignal<Angle> clapperPositionRotations;
  private final StatusSignal<AngularVelocity> clapperVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> clapperAppliedVolts;
  private final StatusSignal<Current> clapperSupplyCurrentAmps;
  private final StatusSignal<Current> clapperTorqueCurrentAmps;
  private final StatusSignal<Temperature> clapperTemperatureCelsius;
  private final StatusSignal<Double> clapperPositionSetpointRotations;
  private final StatusSignal<Double> clapperPositionErrorRotations;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;
  private final StatusSignal<Double> rollerVelocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> rollerVelocityErrorRotationsPerSecond;

  private final StatusSignal<Angle> encoderPositionRotations;

  private double clapperGoalRadians;
  private double rollerGoalRadiansPerSecond;

  private VoltageOut voltageRequest;
  private NeutralOut neutralRequest;
  private MotionMagicVelocityVoltage velocityControlRequest;
  private MotionMagicVoltage positionControlRequest;

  private final Alert clapperDisconnectedAlert =
      new Alert("Funnel Clapper Motor Disconnected. Check CAN bus!", AlertType.ERROR);
  private final Alert rollerDisconnectedAlert =
      new Alert("Funnel Roller Motor Disconnected. Check CAN bus!", AlertType.ERROR);

  public FunnelIOTalonFX() {
    this.clapperMotor = new TalonFX(FunnelConstants.CLAPPER_MOTOR_ID);
    this.rollerMotor = new TalonFX(FunnelConstants.ROLLER_MOTOR_ID);
    this.coralSensor = new DigitalInput(FunnelConstants.CORAL_SENSOR_ID);
    this.clapperEncoder = new CANcoder(FunnelConstants.CLAPPER_MOTOR_ID);

    TalonFXConfiguration clapperConfig = new TalonFXConfiguration();
    clapperConfig.Feedback.FeedbackRemoteSensorID = clapperEncoder.getDeviceID();
    clapperConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    clapperConfig.Feedback.SensorToMechanismRatio = FunnelConstants.CLAPPER_MOTOR_GEAR_RATIO;
    clapperConfig.CurrentLimits.withSupplyCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.CLAPPER_SUPPLY_CURRENT_LIMIT());
    clapperConfig.CurrentLimits.withStatorCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.CLAPPER_STATOR_CURRENT_LIMIT());
    clapperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    clapperConfig.Slot0.kP = FunnelConstants.CLAPPER_MOTOR_GAINS.kP().get();
    clapperConfig.Slot0.kD = FunnelConstants.CLAPPER_MOTOR_GAINS.kD().get();
    clapperConfig.Slot0.kS = FunnelConstants.CLAPPER_MOTOR_GAINS.kS().get();
    clapperConfig.Slot0.kV = FunnelConstants.CLAPPER_MOTOR_GAINS.kV().get();
    clapperConfig.Slot0.kA = FunnelConstants.CLAPPER_MOTOR_GAINS.kA().get();
    clapperConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        FunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS().get();
    clapperConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        FunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS().get();
    clapperConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.CLAPPER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get();
    clapperConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.CLAPPER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get();

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.withSupplyCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.ROLLER_SUPPLY_CURRENT_LIMIT());
    rollerConfig.CurrentLimits.withStatorCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.ROLLER_STATOR_CURRENT_LIMIT());
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Feedback.SensorToMechanismRatio = FunnelConstants.ROLLER_MOTOR_GEAR_RATIO;
    rollerConfig.Slot0.kP = FunnelConstants.ROLLER_MOTOR_GAINS.kP().get();
    rollerConfig.Slot0.kD = FunnelConstants.ROLLER_MOTOR_GAINS.kD().get();
    rollerConfig.Slot0.kS = FunnelConstants.ROLLER_MOTOR_GAINS.kS().get();
    rollerConfig.Slot0.kV = FunnelConstants.ROLLER_MOTOR_GAINS.kV().get();
    rollerConfig.Slot0.kA = FunnelConstants.ROLLER_MOTOR_GAINS.kA().get();
    rollerConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.ROLLER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get();
    rollerConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.ROLLER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get();

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.MagnetOffset =
        FunnelConstants.CANCODER_ABSOLUTE_OFFSET_ROTATIONS.get();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    clapperMotor.getConfigurator().apply(clapperConfig);
    rollerMotor.getConfigurator().apply(rollerConfig);
    clapperEncoder.getConfigurator().apply(canCoderConfig);

    clapperPositionRotations = clapperMotor.getPosition();
    clapperVelocityRotationsPerSecond = clapperMotor.getVelocity();
    clapperAppliedVolts = clapperMotor.getMotorVoltage();
    clapperSupplyCurrentAmps = clapperMotor.getSupplyCurrent();
    clapperTorqueCurrentAmps = clapperMotor.getTorqueCurrent();
    clapperTemperatureCelsius = clapperMotor.getDeviceTemp();
    clapperPositionSetpointRotations = clapperMotor.getClosedLoopReference();
    clapperPositionErrorRotations = clapperMotor.getClosedLoopError();

    rollerPositionRotations = rollerMotor.getPosition();
    rollerVelocityRotationsPerSecond = rollerMotor.getVelocity();
    rollerAppliedVolts = rollerMotor.getMotorVoltage();
    rollerSupplyCurrentAmps = rollerMotor.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerMotor.getTorqueCurrent();
    rollerTemperatureCelsius = rollerMotor.getDeviceTemp();
    rollerVelocitySetpointRotationsPerSecond = rollerMotor.getClosedLoopReference();
    rollerVelocityErrorRotationsPerSecond = rollerMotor.getClosedLoopError();

    encoderPositionRotations = clapperEncoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        clapperPositionRotations,
        clapperVelocityRotationsPerSecond,
        clapperAppliedVolts,
        clapperSupplyCurrentAmps,
        clapperTorqueCurrentAmps,
        clapperTemperatureCelsius,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);

    clapperMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();

    voltageRequest = new VoltageOut(0.0);
    neutralRequest = new NeutralOut();
    velocityControlRequest = new MotionMagicVelocityVoltage(0.0);
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    boolean clapperConnected =
        BaseStatusSignal.refreshAll(
                clapperPositionRotations,
                clapperVelocityRotationsPerSecond,
                clapperAppliedVolts,
                clapperSupplyCurrentAmps,
                clapperTorqueCurrentAmps,
                clapperTemperatureCelsius,
                clapperPositionSetpointRotations,
                clapperPositionErrorRotations)
            .isOK();

    boolean rollerConnected =
        BaseStatusSignal.refreshAll(
                rollerPositionRotations,
                rollerVelocityRotationsPerSecond,
                rollerAppliedVolts,
                rollerSupplyCurrentAmps,
                rollerTorqueCurrentAmps,
                rollerTemperatureCelsius,
                rollerVelocitySetpointRotationsPerSecond,
                rollerVelocityErrorRotationsPerSecond)
            .isOK();

    clapperDisconnectedAlert.set(!clapperConnected);
    rollerDisconnectedAlert.set(!rollerConnected);

    encoderPositionRotations.refresh();

    inputs.clapperPositionRadians =
        Units.rotationsToRadians(clapperPositionRotations.getValueAsDouble());
    inputs.clapperVelocityRadiansPerSecond =
        Units.rotationsToRadians(clapperVelocityRotationsPerSecond.getValueAsDouble());
    inputs.clapperGoalRadians = clapperGoalRadians;
    inputs.clapperAppliedVolts = clapperAppliedVolts.getValueAsDouble();
    inputs.clapperSupplyCurrentAmps = clapperSupplyCurrentAmps.getValueAsDouble();
    inputs.clapperTorqueCurrentAmps = clapperTorqueCurrentAmps.getValueAsDouble();
    inputs.clapperTemperatureCelsius = clapperTemperatureCelsius.getValueAsDouble();
    inputs.clapperPositionSetpointRadians = clapperPositionSetpointRotations.getValueAsDouble();
    inputs.clapperPositionErrorRadians = clapperPositionErrorRotations.getValueAsDouble();
    inputs.encoderPositionRadians =
        Units.rotationsToRadians(encoderPositionRotations.getValueAsDouble());

    inputs.rollerPositionRadians =
        Units.rotationsToRadians(rollerPositionRotations.getValueAsDouble());
    inputs.rollerVelocityRadiansPerSecond =
        Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rollerGoalRadiansPerSecond = rollerGoalRadiansPerSecond;
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();
    inputs.rollerVelocitySetpointRadiansPerSecond =
        rollerVelocitySetpointRotationsPerSecond.getValueAsDouble();
    inputs.rollerVelocityErrorRadiansPerSecond =
        rollerVelocityErrorRotationsPerSecond.getValueAsDouble();

    inputs.hasCoral = coralSensor.get();
  }

  @Override
  public void setClapperVoltage(double volts) {
    clapperMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setClapperPosition(double radians) {
    clapperGoalRadians = radians;
    clapperMotor.setControl(
        positionControlRequest.withPosition(Units.radiansToRotations(radians)).withEnableFOC(true));
  }

  @Override
  public void setRollerVelocity(double radiansPerSecond) {
    rollerGoalRadiansPerSecond = radiansPerSecond;
    rollerMotor.setControl(
        velocityControlRequest
            .withVelocity(Units.radiansToRotations(radiansPerSecond))
            .withEnableFOC(true));
  }

  @Override
  public void stopRoller() {
    rollerMotor.setControl(neutralRequest);
  }

  @Override
  public boolean atClapperGoal() {
    return Math.abs(
            clapperGoalRadians
                - Units.rotationsToRadians(clapperPositionRotations.getValueAsDouble()))
        < FunnelConstants.CLAPPER_MOTOR_CONSTRAINTS.GOAL_TOLERANCE().get();
  }

  @Override
  public boolean atRollerGoal() {
    return Math.abs(
            rollerGoalRadiansPerSecond
                - Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble()))
        < FunnelConstants.ROLLER_MOTOR_CONSTRAINTS.GOAL_TOLERANCE().get();
  }
}
