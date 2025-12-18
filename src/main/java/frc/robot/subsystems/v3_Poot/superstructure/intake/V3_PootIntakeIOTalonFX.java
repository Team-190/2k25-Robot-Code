package frc.robot.subsystems.v3_Poot.superstructure.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import java.util.ArrayList;

public class V3_PootIntakeIOTalonFX implements V3_PootIntakeIO {
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

  private final TalonFX rollerTalonFXOuter;
  private final TalonFX rollerTalonFXInner;

  private final StatusSignal<Angle> rollerInnerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerInnerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerInnerAppliedVoltage;
  private final StatusSignal<Current> rollerInnerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerInnerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerInnerTemperatureCelsius;

  private final StatusSignal<Angle> rollerOuterPositionRotations;
  private final StatusSignal<AngularVelocity> rollerOuterVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerOuterAppliedVoltage;
  private final StatusSignal<Current> rollerOuterSupplyCurrentAmps;
  private final StatusSignal<Current> rollerOuterTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerOuterTemperatureCelsius;

  private final VoltageOut rollerInnerVoltageRequest;
  private final VoltageOut rollerOuterVoltageRequest;

  private final TalonFXConfiguration rollerInnerConfig;
  private final TalonFXConfiguration rollerOuterConfig;

  private final CANrange leftCANrange;
  private final CANrange rightCANrange;

  private final StatusSignal<Boolean> leftCANrangeStatusSignal;
  private final StatusSignal<Boolean> rightCANrangeStatusSignal;

  private StatusSignal<?>[] statusSignals;

  public V3_PootIntakeIOTalonFX() {
    pivotTalonFX = new TalonFX(V3_PootIntakeConstants.PIVOT_CAN_ID);
    rollerTalonFXOuter = new TalonFX(V3_PootIntakeConstants.ROLLER_CAN_ID_OUTER);
    rollerTalonFXInner = new TalonFX(V3_PootIntakeConstants.ROLLER_CAN_ID_INNER);

    leftCANrange = new CANrange(V3_PootIntakeConstants.LEFT_SENSOR_CAN_ID);
    rightCANrange = new CANrange(V3_PootIntakeConstants.RIGHT_SENSOR_CAN_ID);

    pivotConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit =
        V3_PootIntakeConstants.CURRENT_LIMITS.PIVOT_SUPPLY_CURRENT_LIMIT();
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit =
        V3_PootIntakeConstants.CURRENT_LIMITS.PIVOT_STATOR_CURRENT_LIMIT();
    pivotConfig.Feedback.SensorToMechanismRatio =
        V3_PootIntakeConstants.PIVOT_PARAMS.PIVOT_GEAR_RATIO();
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V3_PootIntakeConstants.PIVOT_PARAMS.MAX_ANGLE().getRotations();
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V3_PootIntakeConstants.PIVOT_PARAMS.MIN_ANGLE().getRotations();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.Slot0 =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKP(V3_PootIntakeConstants.PIVOT_GAINS.kP().get())
            .withKD(V3_PootIntakeConstants.PIVOT_GAINS.kD().get())
            .withKA(V3_PootIntakeConstants.PIVOT_GAINS.kA().get())
            .withKV(V3_PootIntakeConstants.PIVOT_GAINS.kV().get())
            .withKS(V3_PootIntakeConstants.PIVOT_GAINS.kS().get())
            .withKG(V3_PootIntakeConstants.PIVOT_GAINS.kG().get());
    pivotConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(
                AngularVelocity.ofRelativeUnits(
                    V3_PootIntakeConstants.PIVOT_CONSTRAINTS
                        .CRUISING_VELOCITY_RADIANS_PER_SECOND()
                        .get(),
                    RadiansPerSecond))
            .withMotionMagicAcceleration(
                AngularAcceleration.ofRelativeUnits(
                    V3_PootIntakeConstants.PIVOT_CONSTRAINTS
                        .MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED()
                        .get(),
                    RadiansPerSecondPerSecond));

    tryUntilOk(5, () -> pivotTalonFX.getConfigurator().apply(pivotConfig, 0.25));

    pivotPositionRotations = pivotTalonFX.getPosition();
    pivotVelocityRotationsPerSecond = pivotTalonFX.getVelocity();
    pivotAppliedVoltage = pivotTalonFX.getMotorVoltage();
    pivotSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
    pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
    pivotTemperatureCelsius = pivotTalonFX.getDeviceTemp();

    pivotPositionSetpoint = pivotTalonFX.getClosedLoopReference();
    pivotPositionError = pivotTalonFX.getClosedLoopError();

    leftCANrangeStatusSignal = leftCANrange.getIsDetected();
    rightCANrangeStatusSignal = rightCANrange.getIsDetected();

    rollerInnerConfig = new TalonFXConfiguration();
    rollerInnerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerInnerConfig.CurrentLimits.SupplyCurrentLimit =
        V3_PootIntakeConstants.CURRENT_LIMITS.INNER_ROLLER_SUPPLY_CURRENT_LIMIT();
    rollerInnerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerInnerConfig.CurrentLimits.StatorCurrentLimit =
        V3_PootIntakeConstants.CURRENT_LIMITS.INNER_ROLLER_STATOR_CURRENT_LIMIT();
    rollerInnerConfig.Feedback.SensorToMechanismRatio =
        V3_PootIntakeConstants.ROLLER_PARAMS.PIVOT_GEAR_RATIO();
    rollerInnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerInnerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rollerOuterConfig = new TalonFXConfiguration();
    rollerOuterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerOuterConfig.CurrentLimits.SupplyCurrentLimit =
        V3_PootIntakeConstants.CURRENT_LIMITS.OUTER_ROLLER_SUPPLY_CURRENT_LIMIT();
    rollerOuterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerOuterConfig.CurrentLimits.StatorCurrentLimit =
        V3_PootIntakeConstants.CURRENT_LIMITS.OUTER_ROLLER_STATOR_CURRENT_LIMIT();
    rollerOuterConfig.Feedback.SensorToMechanismRatio =
        V3_PootIntakeConstants.ROLLER_PARAMS.PIVOT_GEAR_RATIO();
    rollerOuterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerOuterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> rollerTalonFXOuter.getConfigurator().apply(rollerOuterConfig, 0.25));
    tryUntilOk(5, () -> rollerTalonFXInner.getConfigurator().apply(rollerInnerConfig, 0.25));

    rollerInnerPositionRotations = rollerTalonFXInner.getPosition();
    rollerInnerVelocityRotationsPerSecond = rollerTalonFXInner.getVelocity();
    rollerInnerAppliedVoltage = rollerTalonFXInner.getMotorVoltage();
    rollerInnerSupplyCurrentAmps = rollerTalonFXInner.getSupplyCurrent();
    rollerInnerTorqueCurrentAmps = rollerTalonFXInner.getTorqueCurrent();
    rollerInnerTemperatureCelsius = rollerTalonFXInner.getDeviceTemp();

    rollerOuterPositionRotations = rollerTalonFXOuter.getPosition();
    rollerOuterVelocityRotationsPerSecond = rollerTalonFXOuter.getVelocity();
    rollerOuterAppliedVoltage = rollerTalonFXOuter.getMotorVoltage();
    rollerOuterSupplyCurrentAmps = rollerTalonFXOuter.getSupplyCurrent();
    rollerOuterTorqueCurrentAmps = rollerTalonFXOuter.getTorqueCurrent();
    rollerOuterTemperatureCelsius = rollerTalonFXOuter.getDeviceTemp();

    pivotVoltageRequest = new VoltageOut(0);
    pivotMotionMagicRequest =
        new MotionMagicVoltage(V3_PootIntakeConstants.PIVOT_PARAMS.MIN_ANGLE().getMeasure());

    rollerInnerVoltageRequest = new VoltageOut(0);
    rollerOuterVoltageRequest = new VoltageOut(0);

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(pivotPositionRotations);
    signalsList.add(pivotVelocityRotationsPerSecond);
    signalsList.add(pivotAppliedVoltage);
    signalsList.add(pivotSupplyCurrentAmps);
    signalsList.add(pivotTorqueCurrentAmps);
    signalsList.add(pivotTemperatureCelsius);
    signalsList.add(rollerInnerPositionRotations);
    signalsList.add(rollerOuterPositionRotations);
    signalsList.add(rollerInnerVelocityRotationsPerSecond);
    signalsList.add(rollerOuterVelocityRotationsPerSecond);
    signalsList.add(rollerInnerAppliedVoltage);
    signalsList.add(rollerOuterAppliedVoltage);
    signalsList.add(rollerInnerSupplyCurrentAmps);
    signalsList.add(rollerOuterSupplyCurrentAmps);
    signalsList.add(rollerInnerTorqueCurrentAmps);
    signalsList.add(rollerOuterTorqueCurrentAmps);
    signalsList.add(rollerInnerTemperatureCelsius);
    signalsList.add(rollerOuterTemperatureCelsius);
    signalsList.add(leftCANrangeStatusSignal);
    signalsList.add(rightCANrangeStatusSignal);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(50, statusSignals);

    pivotTalonFX.optimizeBusUtilization();
    rollerTalonFXInner.optimizeBusUtilization();
    rollerTalonFXOuter.optimizeBusUtilization();
    leftCANrange.optimizeBusUtilization();
    rightCANrange.optimizeBusUtilization();

    PhoenixUtil.registerSignals(false, statusSignals);

    pivotTalonFX.setPosition(Units.degreesToRotations(48));
  }

  /**
   * Updates the inputs of the subsystem.
   *
   * <p>This function is called by the robot periodic loop and should update all of the inputs of
   * the subsystem. The inputs should be updated from the CAN bus and other sensors.
   *
   * @param inputs The inputs of the subsystem.
   */
  @Override
  public void updateInputs(V3_PootIntakeIOInputs inputs) {
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

    inputs.rollerInnerPosition = new Rotation2d(rollerInnerPositionRotations.getValue());
    inputs.rollerInnerVelocityRadiansPerSecond =
        rollerInnerVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);
    inputs.rollerInnerAppliedVolts = rollerInnerAppliedVoltage.getValueAsDouble();
    inputs.rollerInnerSupplyCurrentAmps = rollerInnerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerInnerTorqueCurrentAmps = rollerInnerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerInnerTemperatureCelsius = rollerInnerTemperatureCelsius.getValueAsDouble();

    inputs.rollerOuterPosition = new Rotation2d(rollerOuterPositionRotations.getValue());
    inputs.rollerOuterVelocityRadiansPerSecond =
        rollerOuterVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);
    inputs.rollerOuterAppliedVolts = rollerOuterAppliedVoltage.getValueAsDouble();
    inputs.rollerOuterSupplyCurrentAmps = rollerOuterSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerOuterTorqueCurrentAmps = rollerOuterTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerOuterTemperatureCelsius = rollerOuterTemperatureCelsius.getValueAsDouble();

    inputs.leftCANRange = leftCANrangeStatusSignal.getValue();
    // result = condition ? true : false;
    inputs.rightCANRange = rightCANrangeStatusSignal.getValue();
  }

  /**
   * Sets the voltage for the intake pivot motor.
   *
   * <p>This method is used to set the voltage for the intake pivot motor in open-loop mode. It sets
   * the isClosedLoop flag to false, and then calls the setPivotVoltage method of the IO interface.
   *
   * @param volts The voltage to set for the intake pivot motor.
   */
  public void setPivotVoltage(double volts) {
    pivotTalonFX.setControl(pivotVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /**
   * Sets the voltage for the inner manipulator roller.
   *
   * <p>This method is used to set the voltage for the inner manipulator roller in open-loop mode.
   * It sets the isClosedLoop flag to false, and then calls the setControl method of the TalonFX
   * object with the voltage request.
   *
   * @param volts The voltage to set for the inner manipulator roller.
   */
  public void setInnerRollerVoltage(double volts) {
    rollerTalonFXInner.setControl(rollerInnerVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /**
   * Sets the voltage for the outer manipulator roller.
   *
   * <p>This method is used to set the voltage for the outer manipulator roller in open-loop mode.
   * It sets the isClosedLoop flag to false, and then calls the setControl method of the TalonFX
   * object with the voltage request.
   *
   * @param volts The voltage to set for the outer manipulator roller.
   */
  public void setOuterRollerVoltage(double volts) {
    rollerTalonFXOuter.setControl(rollerOuterVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /**
   * Sets the position of the intake pivot motor using Motion Magic.
   *
   * <p>This method is used to set the position of the intake pivot motor using Motion Magic. It
   * sets the isClosedLoop flag to true, and then calls the setControl method of the TalonFX object
   * with the position request.
   *
   * @param position The desired position of the intake pivot motor.
   */
  @Override
  public void setPivotGoal(Rotation2d position) {

    pivotTalonFX.setControl(
        pivotMotionMagicRequest.withPosition(position.getMeasure()).withEnableFOC(true));
  }

  public void updateIntakeGains(double kP, double kD, double kS, double kG, double kV, double kA) {
    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kD = kD;
    pivotConfig.Slot0.kS = kS;
    pivotConfig.Slot0.kG = kG;
    pivotConfig.Slot0.kV = kV;
    pivotConfig.Slot0.kA = kA;

    tryUntilOk(5, () -> pivotTalonFX.getConfigurator().apply(pivotConfig, 0.25));
  }

  public void updateIntakeConstraints(
      double maxVelocityRadiansPerSecond, double maxAccelerationRadiansPerSecondSquared) {
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        AngularVelocity.ofRelativeUnits(maxVelocityRadiansPerSecond, RadiansPerSecond)
            .in(RotationsPerSecond);
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        AngularAcceleration.ofRelativeUnits(
                maxAccelerationRadiansPerSecondSquared, RadiansPerSecondPerSecond)
            .in(RotationsPerSecondPerSecond);

    tryUntilOk(5, () -> pivotTalonFX.getConfigurator().apply(pivotConfig, 0.25));
  }
}
