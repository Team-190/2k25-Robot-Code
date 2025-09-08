package frc.robot.subsystems.v3_Epsilon.superstructure.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class V3_EpsilonIntakeIOTalonFX implements V3_EpsilonIntakeIO {
  private final TalonFX pivotTalonFX;

  private final StatusSignal<Angle> pivotPositionRotations;
  private final StatusSignal<AngularVelocity> pivotVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> pivotAppliedVoltage;
  private final StatusSignal<Current> pivotSupplyCurrentAmps;
  private final StatusSignal<Current> pivotTorqueCurrentAmps;
  private final StatusSignal<Temperature> pivotTemperatureCelsius;

  private final StatusSignal<Double> pivotPositionSetpoint;
  private final StatusSignal<Double> pivotPositionError;

  private final VoltageOut pivotVoltageRequest;
  private final MotionMagicVoltage pivotMotionMagicRequest;

  private final TalonFXConfiguration pivotConfig;

  private final TalonFX rollerTalonFXBottom;
  private final TalonFX rollerTalonFXTop;

  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private final VoltageOut rollerVoltageRequest;

  private final TalonFXConfiguration rollerConfig;

  private final CANrange leftCANrange;
  private final CANrange rightCANrange;

  private final StatusSignal<Distance> leftCANrangeStatusSignal;
  private final StatusSignal<Distance> rightCANrangeStatusSignal;

  public V3_EpsilonIntakeIOTalonFX() {
    pivotTalonFX = new TalonFX(V3_EpsilonIntakeConstants.PIVOT_CAN_ID);
    rollerTalonFXBottom = new TalonFX(V3_EpsilonIntakeConstants.ROLLER_CAN_ID_BOTTOM);
    rollerTalonFXTop = new TalonFX(V3_EpsilonIntakeConstants.ROLLER_CAN_ID_TOP);

    leftCANrange = new CANrange(V3_EpsilonIntakeConstants.LEFT_SENSOR_CAN_ID);
    rightCANrange = new CANrange(V3_EpsilonIntakeConstants.RIGHT_SENSOR_CAN_ID);

    pivotConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit =
        V3_EpsilonIntakeConstants.CURRENT_LIMITS.PIVOT_SUPPLY_CURRENT_LIMIT();
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit =
        V3_EpsilonIntakeConstants.CURRENT_LIMITS.PIVOT_STATOR_CURRENT_LIMIT();
    pivotConfig.Feedback.SensorToMechanismRatio =
        V3_EpsilonIntakeConstants.PIVOT_PARAMS.PIVOT_GEAR_RATIO();
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V3_EpsilonIntakeConstants.PIVOT_PARAMS.MIN_ANGLE().getRotations();
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V3_EpsilonIntakeConstants.PIVOT_PARAMS.MAX_ANGLE().getRotations();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.Slot0 =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKP(V3_EpsilonIntakeConstants.PIVOT_GAINS.kP())
            .withKD(V3_EpsilonIntakeConstants.PIVOT_GAINS.kD())
            .withKA(V3_EpsilonIntakeConstants.PIVOT_GAINS.kA())
            .withKV(V3_EpsilonIntakeConstants.PIVOT_GAINS.kV())
            .withKS(V3_EpsilonIntakeConstants.PIVOT_GAINS.kS())
            .withKG(V3_EpsilonIntakeConstants.PIVOT_GAINS.kG());
    pivotConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(
                AngularVelocity.ofRelativeUnits(
                    V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS
                        .CRUISING_VELOCITY_RADIANS_PER_SECOND(),
                    RadiansPerSecond))
            .withMotionMagicAcceleration(
                AngularAcceleration.ofRelativeUnits(
                    V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS
                        .MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED(),
                    RadiansPerSecondPerSecond));

    tryUntilOk(5, () -> pivotTalonFX.getConfigurator().apply(pivotConfig, 0.25));

    pivotPositionRotations = pivotTalonFX.getPosition();
    pivotVelocityRotationsPerSecond = pivotTalonFX.getVelocity();
    pivotAppliedVoltage = pivotTalonFX.getSupplyVoltage();
    pivotSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
    pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
    pivotTemperatureCelsius = pivotTalonFX.getDeviceTemp();

    pivotPositionSetpoint = pivotTalonFX.getClosedLoopReference();
    pivotPositionError = pivotTalonFX.getClosedLoopError();

    leftCANrangeStatusSignal = leftCANrange.getDistance();
    rightCANrangeStatusSignal = rightCANrange.getDistance();

    rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit =
        V3_EpsilonIntakeConstants.CURRENT_LIMITS.ROLLER_SUPPLY_CURRENT_LIMIT();
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit =
        V3_EpsilonIntakeConstants.CURRENT_LIMITS.ROLLER_STATOR_CURRENT_LIMIT();
    rollerConfig.Feedback.SensorToMechanismRatio =
        V3_EpsilonIntakeConstants.ROLLER_PARAMS.PIVOT_GEAR_RATIO();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> rollerTalonFXBottom.getConfigurator().apply(rollerConfig, 0.25));
    tryUntilOk(5, () -> rollerTalonFXTop.getConfigurator().apply(rollerConfig, 0.25));

    rollerPositionRotations = pivotTalonFX.getPosition();
    rollerVelocityRotationsPerSecond = pivotTalonFX.getVelocity();
    rollerAppliedVoltage = pivotTalonFX.getSupplyVoltage();
    rollerSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
    rollerTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
    rollerTemperatureCelsius = pivotTalonFX.getDeviceTemp();

    pivotVoltageRequest = new VoltageOut(0);
    pivotMotionMagicRequest =
        new MotionMagicVoltage(V3_EpsilonIntakeConstants.PIVOT_PARAMS.MIN_ANGLE().getMeasure());

    rollerVoltageRequest = new VoltageOut(0);

    PhoenixUtil.registerSignals(
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
        rollerTemperatureCelsius,
        leftCANrangeStatusSignal,
        rightCANrangeStatusSignal);
  }

  @Override
  public void updateInputs(V3_EpsilonIntakeIOInputs inputs) {
    inputs.pivotPosition = new Rotation2d(pivotPositionRotations.getValue());
    inputs.pivotVelocityRadiansPerSecond =
        pivotVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);
    inputs.pivotAppliedVolts = pivotAppliedVoltage.getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotSupplyCurrentAmps.getValueAsDouble();
    inputs.pivotTorqueCurrentAmps = pivotTorqueCurrentAmps.getValueAsDouble();
    inputs.pivotTemperatureCelsius = pivotTemperatureCelsius.getValueAsDouble();

    inputs.pivotPositionGoal = new Rotation2d(pivotMotionMagicRequest.getPositionMeasure());
    inputs.pivotPositionSetpoint =
        Rotation2d.fromRotations(pivotPositionSetpoint.getValueAsDouble());
    inputs.pivotPositionError = Rotation2d.fromRotations(pivotPositionError.getValueAsDouble());

    inputs.rollerPosition = new Rotation2d(rollerPositionRotations.getValue());
    inputs.rollerVelocityRadiansPerSecond =
        rollerVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();

    inputs.leftHasCoral =
        leftCANrangeStatusSignal.getValueAsDouble() > V3_EpsilonIntakeConstants.INTAKE_CAN_THRESHOLD
            ? true
            : false;
    // result = condition ? true : false;
    inputs.rightHasCoral =
        rightCANrangeStatusSignal.getValueAsDouble()
                > V3_EpsilonIntakeConstants.INTAKE_CAN_THRESHOLD
            ? true
            : false;
  }

  public void setPivotVoltage(double volts) {
    pivotTalonFX.setControl(pivotVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  public void setRollerVoltage(double volts) {
    rollerTalonFXBottom.setControl(rollerVoltageRequest.withOutput(volts).withEnableFOC(true));
    rollerTalonFXTop.setControl(rollerVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  public void setPivotMotionMagic(Rotation2d position) {
    pivotTalonFX.setControl(
        pivotMotionMagicRequest.withPosition(position.getMeasure()).withEnableFOC(true));
  }
}
