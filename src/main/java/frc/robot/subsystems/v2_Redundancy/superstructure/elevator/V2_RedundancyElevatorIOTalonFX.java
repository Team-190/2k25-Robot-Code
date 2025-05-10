package frc.robot.subsystems.v2_Redundancy.superstructure.elevator;

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
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.PhoenixUtil;
import java.util.ArrayList;

public class V2_RedundancyElevatorIOTalonFX implements V2_RedundancyElevatorIO {
  private final TalonFX talonFX;
  private final TalonFX[] followTalonFX;

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

  public V2_RedundancyElevatorIOTalonFX() {
    talonFX = new TalonFX(V2_RedundancyElevatorConstants.ELEVATOR_CAN_ID);
    followTalonFX = new TalonFX[V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1];

    for (int i = 0; i < V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1; i++) {
      followTalonFX[i] = new TalonFX(V2_RedundancyElevatorConstants.ELEVATOR_CAN_ID + i + 1);
    }

    config = new TalonFXConfiguration();
    config.Slot0.kP = V2_RedundancyElevatorConstants.GAINS.kP().get();
    config.Slot0.kD = V2_RedundancyElevatorConstants.GAINS.kD().get();
    config.Slot0.kS = V2_RedundancyElevatorConstants.GAINS.kS().get();
    config.Slot0.kV = V2_RedundancyElevatorConstants.GAINS.kV().get();
    config.Slot0.kA = V2_RedundancyElevatorConstants.GAINS.kA().get();
    config.Slot0.kG = V2_RedundancyElevatorConstants.GAINS.kG().get();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.Slot1.kP = V2_RedundancyElevatorConstants.STOW_GAINS.kP().get();
    config.Slot1.kD = V2_RedundancyElevatorConstants.STOW_GAINS.kD().get();
    config.Slot1.kS = V2_RedundancyElevatorConstants.STOW_GAINS.kS().get();
    config.Slot1.kV = V2_RedundancyElevatorConstants.STOW_GAINS.kV().get();
    config.Slot1.kA = V2_RedundancyElevatorConstants.STOW_GAINS.kA().get();
    config.Slot1.kG = V2_RedundancyElevatorConstants.STOW_GAINS.kG().get();
    config.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    config.CurrentLimits.SupplyCurrentLimit = V2_RedundancyElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = V2_RedundancyElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS()
            / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
            * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS()
            / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
            * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration =
        V2_RedundancyElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().get()
            / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
            * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity =
        V2_RedundancyElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get()
            / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
            * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO;

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

    for (int i = 0; i < V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1; i++) {
      appliedVolts.add(followTalonFX[i].getMotorVoltage());
      supplyCurrentAmps.add(followTalonFX[i].getSupplyCurrent());
      torqueCurrentAmps.add(followTalonFX[i].getTorqueCurrent());
      temperatureCelsius.add(followTalonFX[i].getDeviceTemp());
    }

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(positionRotations);
    signalsList.add(velocityRotationsPerSecond);
    signalsList.add(positionSetpointRotations);
    signalsList.add(positionErrorRotations);
    signalsList.addAll(appliedVolts);
    signalsList.addAll(supplyCurrentAmps);
    signalsList.addAll(torqueCurrentAmps);
    signalsList.addAll(temperatureCelsius);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(50, statusSignals);

    talonFX.optimizeBusUtilization();
    for (TalonFX follower : followTalonFX) {
      follower.optimizeBusUtilization();
    }

    positionVoltageRequest = new MotionMagicVoltage(0.0);
    voltageRequest = new VoltageOut(0.0);

    PhoenixUtil.registerSignals(false, statusSignals);
  }

  @Override
  public void updateInputs(V2_RedundancyElevatorIOInputs inputs) {
    InternalLoggedTracer.reset();
    inputs.positionMeters =
        (positionRotations.getValueAsDouble() / V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO)
            * Math.PI
            * V2_RedundancyElevatorConstants.DRUM_RADIUS
            * 2;
    inputs.velocityMetersPerSecond =
        (velocityRotationsPerSecond.getValueAsDouble() / V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO)
            * Math.PI
            * V2_RedundancyElevatorConstants.DRUM_RADIUS
            * 2;
    for (int i = 0; i < V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
      inputs.appliedVolts[i] = appliedVolts.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius.get(i).getValueAsDouble();
    }
    inputs.positionGoalMeters = positionGoalMeters;
    inputs.positionSetpointMeters =
        (positionSetpointRotations.getValueAsDouble() / V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO)
            * Math.PI
            * V2_RedundancyElevatorConstants.DRUM_RADIUS
            * 2;
    inputs.positionErrorMeters =
        (positionErrorRotations.getValueAsDouble() / V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO)
            * Math.PI
            * V2_RedundancyElevatorConstants.DRUM_RADIUS
            * 2;
    InternalLoggedTracer.record("Update Inputs", "Elevator/TalonFX");
  }

  @Override
  public void setVoltage(double volts) {
    InternalLoggedTracer.reset();
    talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
    InternalLoggedTracer.record("Set Voltage", "Elevator/TalonFX");
  }

  @Override
  public void setPosition(double meters) {
    InternalLoggedTracer.reset();
    talonFX.setPosition(
        meters
            / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
            * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO);
    InternalLoggedTracer.record("Set Position", "Elevator/TalonFX");
  }

  @Override
  public void setPositionGoal(double meters) {
    InternalLoggedTracer.reset();
    positionGoalMeters = meters;
    if (meters != 0.0) {
      talonFX.setControl(
          positionVoltageRequest
              .withPosition(
                  meters
                      / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
                      * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO)
              .withSlot(0));
    } else {
      talonFX.setControl(
          positionVoltageRequest
              .withPosition(
                  meters
                      / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
                      * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO)
              .withSlot(1));
    }
    InternalLoggedTracer.record("Set Position Goal", "Elevator/TalonFX");
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    InternalLoggedTracer.reset();
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
    InternalLoggedTracer.record("Set PID", "Elevator/TalonFX");
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    InternalLoggedTracer.reset();
    config.MotionMagic.MotionMagicAcceleration =
        maxAcceleration
            / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
            * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity =
        cruisingVelocity
            / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS)
            * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO;
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
    for (TalonFX follow : followTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follow.getConfigurator().apply(config));
    }
    InternalLoggedTracer.record("Set Constraints", "Elevator/TalonFX");
  }
}
