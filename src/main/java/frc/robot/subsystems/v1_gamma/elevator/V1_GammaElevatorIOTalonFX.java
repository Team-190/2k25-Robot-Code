package frc.robot.subsystems.v1_gamma.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class V1_GammaElevatorIOTalonFX implements V1_GammaElevatorIO {
  private final TalonFX talonFX;
  private final TalonFX[] followTalonFX = new TalonFX[3];

  private TalonFXConfiguration config;

  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private double positionGoalMeters;
  private StatusSignal<Double> positionSetpointRotations;
  private StatusSignal<Double> positionErrorRotations;
  private StatusSignal<Voltage>[] appliedVolts;
  private StatusSignal<Current>[] supplyCurrentAmps;
  private StatusSignal<Current>[] torqueCurrentAmps;
  private StatusSignal<Temperature>[] temperatureCelsius;

  private StatusSignal<?>[] statusSignals;

  private MotionMagicVoltage positionVoltageRequest;
  private VoltageOut voltageRequest;

  public V1_GammaElevatorIOTalonFX() {
    talonFX = new TalonFX(V1_GammaElevatorConstants.ELEVATOR_CAN_ID);
    followTalonFX[0] = new TalonFX(V1_GammaElevatorConstants.ELEVATOR_CAN_ID + 1);
    followTalonFX[1] = new TalonFX(V1_GammaElevatorConstants.ELEVATOR_CAN_ID + 2);
    followTalonFX[2] = new TalonFX(V1_GammaElevatorConstants.ELEVATOR_CAN_ID + 3);

    config = new TalonFXConfiguration();
    config.Slot0.kP = V1_GammaElevatorConstants.GAINS.kP().get();
    config.Slot0.kD = V1_GammaElevatorConstants.GAINS.kD().get();
    config.Slot0.kS = V1_GammaElevatorConstants.GAINS.kS().get();
    config.Slot0.kV = V1_GammaElevatorConstants.GAINS.kV().get();
    config.Slot0.kA = V1_GammaElevatorConstants.GAINS.kA().get();
    config.Slot0.kG = V1_GammaElevatorConstants.GAINS.kG().get();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.CurrentLimits.SupplyCurrentLimit =
        V1_GammaElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit =
        V1_GammaElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = V1_GammaElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS()
            * V1_GammaElevatorConstants.ELEVATOR_GEAR_RATIO
            / (2 * Math.PI * V1_GammaElevatorConstants.DRUM_RADIUS);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS()
            * V1_GammaElevatorConstants.ELEVATOR_GEAR_RATIO
            / (2 * Math.PI * V1_GammaElevatorConstants.DRUM_RADIUS);

    config.MotionMagic.MotionMagicAcceleration =
        V1_GammaElevatorConstants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        V1_GammaElevatorConstants.CONSTRAINTS.cruisingVelocityRadiansPerSecond().get();

    talonFX.getConfigurator().apply(config);
    for (TalonFX follower : followTalonFX) {
      follower.getConfigurator().apply(config);
      follower.setControl(
          new Follower(
              talonFX.getDeviceID(),
              follower.getDeviceID() % 2 == 0 ? false : true)); // odd IDs are inverted
    }

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
    positionGoalMeters = 0.0;
    positionSetpointRotations = talonFX.getClosedLoopReference();
    positionErrorRotations = talonFX.getClosedLoopError();

    for (int i = 0; i < V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
      appliedVolts[i] = followTalonFX[i].getMotorVoltage();
      supplyCurrentAmps[i] = followTalonFX[i].getSupplyCurrent();
      torqueCurrentAmps[i] = followTalonFX[i].getTorqueCurrent();
      temperatureCelsius[i] = followTalonFX[i].getDeviceTemp();
    }

    statusSignals =
        new StatusSignal<?>[] {
          positionRotations,
          velocityRotationsPerSecond,
          positionSetpointRotations,
          positionErrorRotations,
          appliedVolts[0],
          supplyCurrentAmps[0],
          torqueCurrentAmps[0],
          temperatureCelsius[0],
          appliedVolts[1],
          supplyCurrentAmps[1],
          torqueCurrentAmps[1],
          temperatureCelsius[1],
          appliedVolts[2],
          supplyCurrentAmps[2],
          torqueCurrentAmps[2],
          temperatureCelsius[2],
          appliedVolts[3],
          supplyCurrentAmps[3],
          torqueCurrentAmps[3],
          temperatureCelsius[3]
        };

    BaseStatusSignal.setUpdateFrequencyForAll(50, statusSignals);

    talonFX.optimizeBusUtilization();
    for (TalonFX follow : followTalonFX) {
      follow.optimizeBusUtilization();
    }

    positionVoltageRequest = new MotionMagicVoltage(0.0);
    voltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(statusSignals).isOK();

    inputs.positionMeters =
        positionRotations.getValueAsDouble() * Math.PI * V1_GammaElevatorConstants.DRUM_RADIUS * 2;
    inputs.velocityMetersPerSecond =
        velocityRotationsPerSecond.getValueAsDouble()
            * Math.PI
            * V1_GammaElevatorConstants.DRUM_RADIUS
            * 2;
    inputs.positionGoalMeters = positionGoalMeters;
    inputs.positionSetpointMeters =
        positionSetpointRotations.getValueAsDouble()
            * Math.PI
            * V1_GammaElevatorConstants.DRUM_RADIUS
            * 2;
    inputs.positionErrorMeters =
        positionErrorRotations.getValueAsDouble()
            * Math.PI
            * V1_GammaElevatorConstants.DRUM_RADIUS
            * 2;

    for (int i = 0; i < V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
      inputs.appliedVolts[i] = appliedVolts[i].getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps[i].getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps[i].getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius[i].getValueAsDouble();
    }
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPosition(double meters) {
    talonFX.setPosition(meters / (2 * Math.PI * V1_GammaElevatorConstants.DRUM_RADIUS));
  }

  @Override
  public void setPositionGoal(double meters) {
    positionGoalMeters = meters;
    talonFX.setControl(
        positionVoltageRequest.withPosition(
            meters / (2 * Math.PI * V1_GammaElevatorConstants.DRUM_RADIUS)));
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    TalonFXConfiguration newGains =
        new TalonFXConfiguration() {
          {
            Slot0.kP = kP;
            Slot0.kD = kD;
            Slot0.kS = kS;
            Slot0.kV = kV;
            Slot0.kA = kA;
            Slot0.kG = kG;
          }
        };
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(newGains));
    for (TalonFX follow : followTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follow.getConfigurator().apply(newGains));
    }
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    TalonFXConfiguration newConstraints =
        new TalonFXConfiguration() {
          {
            MotionMagic.MotionMagicAcceleration = maxAcceleration;
            MotionMagic.MotionMagicCruiseVelocity = cruisingVelocity;
          }
        };
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(newConstraints));
    for (TalonFX follow : followTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follow.getConfigurator().apply(newConstraints));
    }
  }
}
