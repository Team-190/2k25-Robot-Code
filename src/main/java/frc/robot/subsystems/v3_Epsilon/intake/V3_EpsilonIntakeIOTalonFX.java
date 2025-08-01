package frc.robot.subsystems.v3_Epsilon.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
  private final NeutralOut rollerNeutralRequest;

  private final TalonFXConfiguration rollerConfig;

  public V3_EpsilonIntakeIOTalonFX() {
    pivotTalonFX = new TalonFX(V3_EpsilonIntakeConstants.PIVOT_CAN_ID);
    rollerTalonFX = new TalonFX(V3_EpsilonIntakeConstants.ROLLER_CAN_ID);

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

    tryUntilOk(5, () -> rollerTalonFX.getConfigurator().apply(rollerConfig, 0.25));

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
    rollerNeutralRequest = new NeutralOut();

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
        rollerTemperatureCelsius);
  }
}
