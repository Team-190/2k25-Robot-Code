package frc.robot.subsystems.v3_Epsilon.superstructure.manipulator;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
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

public class V3_EpsilonManipulatorIOTalonFX implements V3_EpsilonManipulatorIO {

  private final TalonFX armTalonFX;
  private final StatusSignal<Angle> armPositionRotations;
  private final StatusSignal<AngularVelocity> armVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> armAppliedVoltage;
  private final StatusSignal<Current> armSupplyCurrentAmps;
  private final StatusSignal<Current> armTorqueCurrentAmps;
  private final StatusSignal<Temperature> armTemperatureCelsius;
  private final StatusSignal<Double> armPositionSetpointRotations;
  private final StatusSignal<Double> armPositionErrorRotations;

  private final VoltageOut armVoltageRequest;
  private final MotionMagicVoltage armMotionMagicRequest;

  private final TalonFXConfiguration armConfig;

  private final TalonFX rollerTalonFX;
  private final StatusSignal<Angle> rollerPositionRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private StatusSignal<?>[] statusSignals;

  private final VoltageOut rollerVoltageRequest;
  private final TalonFXConfiguration rollerConfig;

  private final CANrange canRange;
  private final StatusSignal<Boolean> isDetected;

  public V3_EpsilonManipulatorIOTalonFX() {
    armTalonFX = new TalonFX(V3_EpsilonManipulatorConstants.ARM_CAN_ID);
    rollerTalonFX = new TalonFX(V3_EpsilonManipulatorConstants.ROLLER_CAN_ID);
    canRange = new CANrange(V3_EpsilonManipulatorConstants.CAN_RANGE_ID);

    armConfig = new TalonFXConfiguration();

    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armConfig.CurrentLimits.SupplyCurrentLimit =
        V3_EpsilonManipulatorConstants.CURRENT_LIMITS.MANIPULATOR_SUPPLY_CURRENT_LIMIT();
    armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armConfig.CurrentLimits.StatorCurrentLimit =
        V3_EpsilonManipulatorConstants.CURRENT_LIMITS.MANIPULATOR_STATOR_CURRENT_LIMIT();
    armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    armConfig.Feedback.SensorToMechanismRatio =
        V3_EpsilonManipulatorConstants.ARM_PARAMETERS.GEAR_RATIO();
    armConfig.Slot0 =
        Slot0Configs.from(V3_EpsilonManipulatorConstants.EMPTY_GAINS.toTalonFXSlotConfigs());
    armConfig.Slot1 =
        Slot1Configs.from(V3_EpsilonManipulatorConstants.CORAL_GAINS.toTalonFXSlotConfigs());
    armConfig.Slot2 =
        Slot2Configs.from(V3_EpsilonManipulatorConstants.ALGAE_GAINS.toTalonFXSlotConfigs());
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.ClosedLoopGeneral.ContinuousWrap = true;
    armConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                AngularAcceleration.ofRelativeUnits(
                    V3_EpsilonManipulatorConstants.CONSTRAINTS
                        .maxAccelerationRotationsPerSecondSquared()
                        .get(),
                    RotationsPerSecondPerSecond))
            .withMotionMagicCruiseVelocity(
                AngularVelocity.ofRelativeUnits(
                    V3_EpsilonManipulatorConstants.CONSTRAINTS
                        .cruisingVelocityRotationsPerSecond()
                        .get(),
                    RotationsPerSecond));

    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));

    armPositionRotations = armTalonFX.getPosition();
    armVelocityRotationsPerSecond = armTalonFX.getVelocity();
    armAppliedVoltage = armTalonFX.getMotorVoltage();
    armSupplyCurrentAmps = armTalonFX.getSupplyCurrent();
    armTorqueCurrentAmps = armTalonFX.getTorqueCurrent();
    armTemperatureCelsius = armTalonFX.getDeviceTemp();

    armPositionSetpointRotations = armTalonFX.getClosedLoopReference();
    armPositionErrorRotations = armTalonFX.getClosedLoopError();

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
    armVoltageRequest = new VoltageOut(0);
    armMotionMagicRequest = new MotionMagicVoltage(0);

    isDetected = canRange.getIsDetected();

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(armPositionRotations);
    signalsList.add(armVelocityRotationsPerSecond);
    signalsList.add(armAppliedVoltage);
    signalsList.add(armSupplyCurrentAmps);
    signalsList.add(armTorqueCurrentAmps);
    signalsList.add(armTemperatureCelsius);
    signalsList.add(armPositionSetpointRotations);
    signalsList.add(armPositionErrorRotations);
    signalsList.add(rollerPositionRotations);
    signalsList.add(rollerVelocityRotationsPerSecond);
    signalsList.add(rollerAppliedVoltage);
    signalsList.add(rollerSupplyCurrentAmps);
    signalsList.add(rollerTorqueCurrentAmps);
    signalsList.add(rollerTemperatureCelsius);
    signalsList.add(isDetected);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(50, statusSignals);

    armTalonFX.optimizeBusUtilization();
    rollerTalonFX.optimizeBusUtilization();
    canRange.optimizeBusUtilization();

    PhoenixUtil.registerSignals(false, statusSignals);
  }

  /**
   * Updates the inputs of the manipulator with the current state of the TalonFXs.
   *
   * @param inputs the inputs to update
   */
  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {

    inputs.armPosition = new Rotation2d(armPositionRotations.getValue());
    inputs.armVelocityRadiansPerSecond =
        armVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);
    inputs.armAppliedVolts = armAppliedVoltage.getValueAsDouble();
    inputs.armSupplyCurrentAmps = armSupplyCurrentAmps.getValueAsDouble();
    inputs.armTorqueCurrentAmps = armTorqueCurrentAmps.getValueAsDouble();
    inputs.armTemperatureCelsius = armTemperatureCelsius.getValueAsDouble();

    inputs.rollerPosition = new Rotation2d(rollerPositionRotations.getValue());
    inputs.rollerVelocityRadiansPerSecond =
        Units.rotationsToRadians(rollerVelocityRotationsPerSecond.getValueAsDouble());
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentAmps.getValueAsDouble();
    inputs.rollerTorqueCurrentAmps = rollerTorqueCurrentAmps.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();

    inputs.armPositionGoal = new Rotation2d(armMotionMagicRequest.getPositionMeasure());
    inputs.armPositionSetpoint =
        Rotation2d.fromRotations(armPositionSetpointRotations.getValueAsDouble());
    inputs.armPositionError =
        Rotation2d.fromRotations(armPositionErrorRotations.getValueAsDouble());

    inputs.canRangeGot = isDetected.getValue();
  }

  /**
   * Sets the voltage of the arm TalonFX to the specified value. The voltage is set in terms of
   * volts, with positive values corresponding to clockwise rotation and negative values
   * corresponding to counterclockwise rotation. This method is used to control the velocity of the
   * arm, which is useful for tasks such as picking up objects or depositing objects.
   *
   * @param volts the voltage to set, in volts
   */
  @Override
  public void setArmVoltage(double volts) {
    armTalonFX.setControl(armVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /**
   * Sets the voltage of the roller TalonFX to the specified value. The voltage is set in terms of
   * volts, with positive values corresponding to clockwise rotation and negative values
   * corresponding to counterclockwise rotation. This method is used to control the velocity of the
   * roller, which is useful for tasks such as picking up objects or depositing objects.
   */
  @Override
  public void setRollerVoltage(double volts) {
    rollerTalonFX.setControl(rollerVoltageRequest.withOutput(volts).withEnableFOC(true));
  }

  /**
   * The position is set in terms of rotations of the TalonFX's motor shaft. This method is used to
   * set the manipulator arm to a specific position, which is useful for tasks such as picking up
   * objects or depositing objects.
   *
   * @param rotation The desired position of the manipulator arm, in terms of rotations of the
   *     TalonFX's motor shaft.
   */
  @Override
  public void setArmGoal(Rotation2d rotation) {
    armTalonFX.setControl(
        armMotionMagicRequest.withPosition(rotation.getRotations()).withEnableFOC(true));
  }

  /**
   * Sets the current slot of the manipulator arm based on the current state of the subsystem. If
   * the subsystem has algae, it sets the slot to 2. If the subsystem has coral, it sets the slot to
   * 1. Otherwise, it sets the slot to 0.
   *
   * @param slot The slot to set the arm to.
   * @throws IllegalArgumentException If the slot is not between 0 and 2, inclusive.
   */
  @Override
  public void setSlot(int slot) {
    if (slot >= 0 && slot <= 2) {
      armTalonFX.setControl(armMotionMagicRequest.withSlot(slot));
    } else {
      throw new IllegalArgumentException("Invalid slot: " + slot);
    }
  }

  public void updateSlot0ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    armConfig.Slot0 =
        new Slot0Configs()
            .withKP(kP)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withKG(kG)
            .withGravityType(GravityTypeValue.Arm_Cosine);
    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));
  }

  public void updateSlot1ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    armConfig.Slot1 =
        new Slot1Configs()
            .withKP(kP)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withKG(kG)
            .withGravityType(GravityTypeValue.Arm_Cosine);
    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));
  }

  public void updateSlot2ArmGains(
      double kP, double kD, double kS, double kV, double kA, double kG) {
    armConfig.Slot2 =
        new Slot2Configs()
            .withKP(kP)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withKG(kG)
            .withGravityType(GravityTypeValue.Arm_Cosine);
    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));
  }

  public void updateArmConstraints(double maxAcceleration, double cruisingVelocity) {
    armConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                AngularAcceleration.ofRelativeUnits(maxAcceleration, RotationsPerSecondPerSecond))
            .withMotionMagicCruiseVelocity(
                AngularVelocity.ofRelativeUnits(cruisingVelocity, RotationsPerSecond));
    tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));
  }
}
