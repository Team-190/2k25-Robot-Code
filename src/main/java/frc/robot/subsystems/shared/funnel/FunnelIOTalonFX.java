package frc.robot.subsystems.shared.funnel;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.shared.drive.TunerConstantsV1_StackUp;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.PhoenixUtil;

public class FunnelIOTalonFX implements FunnelIO {
  private final TalonFX clapDaddyTalonFX;
  private final TalonFX rollerTalonFX;
  private final DigitalInput coralSensor;
  private final CANcoder clapDaddyCANcoder;

  private final TalonFXConfiguration clapDaddyConfig;
  private final TalonFXConfiguration rollerConfig;
  private final CANcoderConfiguration cancoderConfig;

  private final StatusSignal<Angle> clapDaddyPositionRotations;
  private final StatusSignal<AngularVelocity> clapDaddyVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> clapDaddyAppliedVolts;
  private final StatusSignal<Current> clapDaddySupplyCurrentAmps;
  private final StatusSignal<Current> clapDaddyTorqueCurrentAmps;
  private final StatusSignal<Temperature> clapDaddyTemperatureCelsius;
  private final StatusSignal<Double> clapDaddyPositionSetpointRotations;
  private final StatusSignal<Double> clapDaddyPositionErrorRotations;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private final StatusSignal<Angle> cancoderPositionRotations;

  private Rotation2d clapDaddyGoal;

  private MotionMagicVoltage positionControlRequest;
  private VoltageOut voltageRequest;
  private NeutralOut neutralRequest;

  public FunnelIOTalonFX() {
    clapDaddyTalonFX =
        new TalonFX(FunnelConstants.CLAP_DADDY_MOTOR_ID, TunerConstantsV1_StackUp.kCANBus);
    rollerTalonFX = new TalonFX(FunnelConstants.ROLLER_MOTOR_ID);
    coralSensor = new DigitalInput(FunnelConstants.CORAL_SENSOR_ID);
    clapDaddyCANcoder =
        new CANcoder(FunnelConstants.CLAP_DADDY_CANCODER_ID, TunerConstantsV1_StackUp.kCANBus);

    clapDaddyConfig = new TalonFXConfiguration();
    clapDaddyConfig.Feedback.SensorToMechanismRatio = FunnelConstants.CLAP_DADDY_MOTOR_GEAR_RATIO;
    clapDaddyConfig.CurrentLimits.withSupplyCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.CLAP_DADDY_SUPPLY_CURRENT_LIMIT());
    clapDaddyConfig.CurrentLimits.withStatorCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.CLAP_DADDY_STATOR_CURRENT_LIMIT());
    clapDaddyConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    clapDaddyConfig.MotorOutput.Inverted = FunnelConstants.CLAP_DADDY_INVERTED;
    clapDaddyConfig.Slot0.kP = FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kP().get();
    clapDaddyConfig.Slot0.kD = FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kD().get();
    clapDaddyConfig.Slot0.kS = FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kS().get();
    clapDaddyConfig.Slot0.kV = FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kV().get();
    clapDaddyConfig.Slot0.kA = FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kA().get();
    clapDaddyConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(FunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS());
    clapDaddyConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    clapDaddyConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(FunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS());
    clapDaddyConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    clapDaddyConfig.MotionMagic.MotionMagicAcceleration =
        FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get();
    clapDaddyConfig.MotionMagic.MotionMagicCruiseVelocity =
        FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_VELOCITY().get();

    tryUntilOk(5, () -> clapDaddyTalonFX.getConfigurator().apply(clapDaddyConfig, 0.25));

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.withSupplyCurrentLimit(
        FunnelConstants.CURRENT_LIMITS.ROLLER_SUPPLY_CURRENT_LIMIT());
    // rollerConfig.CurrentLimits.withStatorCurrentLimit(
    //     FunnelConstants.CURRENT_LIMITS.ROLLER_STATOR_CURRENT_LIMIT());
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.Feedback.SensorToMechanismRatio = FunnelConstants.ROLLER_MOTOR_GEAR_RATIO;
    rollerConfig.MotorOutput.Inverted = FunnelConstants.ROLLER_INVERTED;

    tryUntilOk(5, () -> rollerTalonFX.getConfigurator().apply(rollerConfig, 0.25));

    cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset =
        FunnelConstants.CANCODER_ABSOLUTE_OFFSET_RADIANS.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    tryUntilOk(5, () -> clapDaddyCANcoder.getConfigurator().apply(cancoderConfig, 0.25));
    tryUntilOk(
        5,
        () ->
            clapDaddyTalonFX.setPosition(
                clapDaddyCANcoder.getPosition().getValueAsDouble()
                    / FunnelConstants.CLAP_DADDY_CANCODER_GEAR_RATIO,
                0.25));

    clapDaddyPositionRotations = clapDaddyTalonFX.getPosition();
    cancoderPositionRotations = clapDaddyCANcoder.getPosition();
    clapDaddyVelocityRotationsPerSecond = clapDaddyTalonFX.getVelocity();
    clapDaddyAppliedVolts = clapDaddyTalonFX.getMotorVoltage();
    clapDaddySupplyCurrentAmps = clapDaddyTalonFX.getSupplyCurrent();
    clapDaddyTorqueCurrentAmps = clapDaddyTalonFX.getTorqueCurrent();
    clapDaddyTemperatureCelsius = clapDaddyTalonFX.getDeviceTemp();
    clapDaddyPositionSetpointRotations = clapDaddyTalonFX.getClosedLoopReference();
    clapDaddyPositionErrorRotations = clapDaddyTalonFX.getClosedLoopError();

    rollerPositionRotations = rollerTalonFX.getPosition();
    rollerVelocityRotationsPerSecond = rollerTalonFX.getVelocity();
    rollerAppliedVolts = rollerTalonFX.getMotorVoltage();
    rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerTalonFX.getTorqueCurrent();
    rollerTemperatureCelsius = rollerTalonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        clapDaddyPositionRotations,
        cancoderPositionRotations,
        clapDaddyVelocityRotationsPerSecond,
        clapDaddyAppliedVolts,
        clapDaddySupplyCurrentAmps,
        clapDaddyTorqueCurrentAmps,
        clapDaddyTemperatureCelsius,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);

    clapDaddyTalonFX.optimizeBusUtilization();
    rollerTalonFX.optimizeBusUtilization();
    clapDaddyCANcoder.optimizeBusUtilization();

    voltageRequest = new VoltageOut(0.0);
    neutralRequest = new NeutralOut();
    positionControlRequest = new MotionMagicVoltage(0.0);

    PhoenixUtil.registerSignals(
        true,
        clapDaddyPositionRotations,
        clapDaddyVelocityRotationsPerSecond,
        clapDaddyAppliedVolts,
        clapDaddySupplyCurrentAmps,
        clapDaddyTorqueCurrentAmps,
        clapDaddyTemperatureCelsius,
        clapDaddyPositionSetpointRotations,
        clapDaddyPositionErrorRotations,
        cancoderPositionRotations);

    PhoenixUtil.registerSignals(
        false,
        rollerPositionRotations,
        rollerVelocityRotationsPerSecond,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemperatureCelsius);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    InternalLoggedTracer.reset();
    inputs.clapDaddyPosition =
        Rotation2d.fromRotations(clapDaddyPositionRotations.getValueAsDouble());
    inputs.clapDaddyAbsolutePosition =
        Rotation2d.fromRotations(
            cancoderPositionRotations.getValueAsDouble()
                / FunnelConstants.CLAP_DADDY_CANCODER_GEAR_RATIO);
    inputs.clapDaddyVelocityRadiansPerSecond =
        Units.rotationsToRadians(clapDaddyVelocityRotationsPerSecond.getValueAsDouble());
    inputs.clapDaddyAppliedVolts = clapDaddyAppliedVolts.getValueAsDouble();
    inputs.clapDaddySupplyCurrentAmps = clapDaddySupplyCurrentAmps.getValueAsDouble();
    inputs.clapDaddyTorqueCurrentAmps = clapDaddyTorqueCurrentAmps.getValueAsDouble();
    inputs.clapDaddyTemperatureCelsius = clapDaddyTemperatureCelsius.getValueAsDouble();
    inputs.clapDaddyGoal = clapDaddyGoal;
    inputs.clapDaddyPositionSetpoint =
        Rotation2d.fromRotations(clapDaddyPositionSetpointRotations.getValueAsDouble());
    inputs.clapDaddyPositionError =
        Rotation2d.fromRotations(clapDaddyPositionErrorRotations.getValueAsDouble());

    inputs.rollerPosition = Rotation2d.fromRotations(rollerPositionRotations.getValueAsDouble());
    inputs.rollerVelocityRadiansPerSecond =
        Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();

    inputs.hasCoral = coralSensor.get();
    InternalLoggedTracer.record("Update Inputs", "Funnel/TalonFX");
  }

  @Override
  public void setClapDaddyVoltage(double volts) {
    InternalLoggedTracer.reset();
    clapDaddyTalonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
    InternalLoggedTracer.record("Set Clap Daddy Voltage", "Funnel/TalonFX");
  }

  @Override
  public void setRollerVoltage(double volts) {
    InternalLoggedTracer.reset();
    rollerTalonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
    InternalLoggedTracer.record("Set Roller Voltage", "Funnel/TalonFX");
  }

  @Override
  public void stopRoller() {
    InternalLoggedTracer.reset();
    rollerTalonFX.setControl(neutralRequest);
    InternalLoggedTracer.record("StopRoller", "Funnel/TalonFX");
  }

  @Override
  public void setClapDaddyGoal(Rotation2d position) {
    InternalLoggedTracer.reset();
    clapDaddyGoal = position;
    clapDaddyTalonFX.setControl(
        positionControlRequest.withPosition(position.getRotations()).withEnableFOC(true));
    InternalLoggedTracer.record("Set Clap Daddy Goal", "Funnel/TalonFX");
  }

  @Override
  public boolean atClapDaddyPositionGoal() {
    return Math.abs(
            clapDaddyGoal.getRadians()
                - Units.rotationsToRadians(clapDaddyPositionRotations.getValueAsDouble()))
        < FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.GOAL_TOLERANCE().get();
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    InternalLoggedTracer.reset();
    clapDaddyConfig.Slot0.kP = kP;
    clapDaddyConfig.Slot0.kD = kD;
    clapDaddyConfig.Slot0.kS = kS;
    clapDaddyConfig.Slot0.kV = kV;
    clapDaddyConfig.Slot0.kA = kA;
    tryUntilOk(5, () -> clapDaddyTalonFX.getConfigurator().apply(clapDaddyConfig, 0.25));
    InternalLoggedTracer.record("Update Gains", "Funnel/TalonFX");
  }

  @Override
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    InternalLoggedTracer.reset();
    clapDaddyConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration;
    clapDaddyConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    tryUntilOk(5, () -> clapDaddyTalonFX.getConfigurator().apply(clapDaddyConfig, 0.25));
    InternalLoggedTracer.record("Update Constraints", "Funnel/TalonFX");
  }
}
