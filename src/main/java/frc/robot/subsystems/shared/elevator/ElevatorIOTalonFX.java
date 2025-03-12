package frc.robot.subsystems.shared.elevator;

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
import java.util.ArrayList;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX talonFX;
  private final TalonFX[] followTalonFX = new TalonFX[3];

  private final TalonFXConfiguration config;

  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private ArrayList<StatusSignal<Voltage>> appliedVolts;
  private ArrayList<StatusSignal<Current>> supplyCurrentAmps;
  private ArrayList<StatusSignal<Current>> torqueCurrentAmps;
  private ArrayList<StatusSignal<Temperature>> temperatureCelsius;
  private double positionGoalMeters;
  private StatusSignal<Double> positionSetpointRotations;
  private StatusSignal<Double> positionErrorRotations;

  private StatusSignal<?>[] statusSignals;

  private MotionMagicVoltage positionVoltageRequest;
  private VoltageOut voltageRequest;

  public ElevatorIOTalonFX() {
    talonFX = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID);
    followTalonFX[0] = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID + 1);
    followTalonFX[1] = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID + 2);
    followTalonFX[2] = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID + 3);

    config = new TalonFXConfiguration();
    config.Slot0.kP = ElevatorConstants.GAINS.kP().get();
    config.Slot0.kD = ElevatorConstants.GAINS.kD().get();
    config.Slot0.kS = ElevatorConstants.GAINS.kS().get();
    config.Slot0.kV = ElevatorConstants.GAINS.kV().get();
    config.Slot0.kA = ElevatorConstants.GAINS.kA().get();
    config.Slot0.kG = ElevatorConstants.GAINS.kG().get();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS()
            / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)
            * ElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS()
            / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)
            * ElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().get()
            / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)
            * ElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get()
            / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)
            * ElevatorConstants.ELEVATOR_GEAR_RATIO;

    talonFX.getConfigurator().apply(config);
    for (TalonFX follower : followTalonFX) {
      follower.getConfigurator().apply(config);
      follower.setControl(
          new Follower(
              talonFX.getDeviceID(),
              follower.getDeviceID() % 2 == 0 ? false : true)); // odd IDs are inverted
    }

    appliedVolts = new ArrayList<>();
    supplyCurrentAmps = new ArrayList<>();
    torqueCurrentAmps = new ArrayList<>();
    temperatureCelsius = new ArrayList<>();

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
    appliedVolts.add(talonFX.getMotorVoltage());
    supplyCurrentAmps.add(talonFX.getSupplyCurrent());
    torqueCurrentAmps.add(talonFX.getTorqueCurrent());
    temperatureCelsius.add(talonFX.getDeviceTemp());
    positionGoalMeters = 0.0;
    positionSetpointRotations = talonFX.getClosedLoopReference();
    positionErrorRotations = talonFX.getClosedLoopError();

    for (int i = 0; i < ElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1; i++) {
      appliedVolts.add(followTalonFX[i].getMotorVoltage());
      supplyCurrentAmps.add(followTalonFX[i].getSupplyCurrent());
      torqueCurrentAmps.add(followTalonFX[i].getTorqueCurrent());
      temperatureCelsius.add(followTalonFX[i].getDeviceTemp());
    }

    statusSignals =
        new StatusSignal<?>[] {
          positionRotations,
          velocityRotationsPerSecond,
          positionSetpointRotations,
          positionErrorRotations,
          appliedVolts.get(0),
          supplyCurrentAmps.get(0),
          torqueCurrentAmps.get(0),
          temperatureCelsius.get(0),
          appliedVolts.get(1),
          supplyCurrentAmps.get(1),
          torqueCurrentAmps.get(1),
          temperatureCelsius.get(1),
          appliedVolts.get(2),
          supplyCurrentAmps.get(2),
          torqueCurrentAmps.get(2),
          temperatureCelsius.get(2),
          appliedVolts.get(3),
          supplyCurrentAmps.get(3),
          torqueCurrentAmps.get(3),
          temperatureCelsius.get(3)
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
        (positionRotations.getValueAsDouble() / ElevatorConstants.ELEVATOR_GEAR_RATIO)
            * Math.PI
            * ElevatorConstants.DRUM_RADIUS
            * 2;
    inputs.velocityMetersPerSecond =
        (velocityRotationsPerSecond.getValueAsDouble() / ElevatorConstants.ELEVATOR_GEAR_RATIO)
            * Math.PI
            * ElevatorConstants.DRUM_RADIUS
            * 2;
    for (int i = 0; i < ElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
      inputs.appliedVolts[i] = appliedVolts.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius.get(i).getValueAsDouble();
    }
    inputs.positionGoalMeters = positionGoalMeters;
    inputs.positionSetpointMeters =
        (positionSetpointRotations.getValueAsDouble() / ElevatorConstants.ELEVATOR_GEAR_RATIO)
            * Math.PI
            * ElevatorConstants.DRUM_RADIUS
            * 2;
    inputs.positionErrorMeters =
        (positionErrorRotations.getValueAsDouble() / ElevatorConstants.ELEVATOR_GEAR_RATIO)
            * Math.PI
            * ElevatorConstants.DRUM_RADIUS
            * 2;
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPosition(double meters) {
    talonFX.setPosition(
        meters
            / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)
            * ElevatorConstants.ELEVATOR_GEAR_RATIO);
  }

  @Override
  public void setPositionGoal(double meters) {
    positionGoalMeters = meters;
    talonFX.setControl(
        positionVoltageRequest.withPosition(
            meters
                / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)
                * ElevatorConstants.ELEVATOR_GEAR_RATIO));
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kG = kG;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
    for (TalonFX follow : followTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follow.getConfigurator().apply(config));
    }
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    config.MotionMagic.MotionMagicAcceleration =
        maxAcceleration
            / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)
            * ElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity =
        cruisingVelocity
            / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)
            * ElevatorConstants.ELEVATOR_GEAR_RATIO;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
    for (TalonFX follow : followTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follow.getConfigurator().apply(config));
    }
  }
}
