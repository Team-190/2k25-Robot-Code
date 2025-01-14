package frc.robot.subsystems.v1_gamma.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX talonFX;
  private final TalonFX[] followTalonFX = new TalonFX[3];

  private StatusSignal<Angle>[] positionRotations;
  private StatusSignal<AngularVelocity>[] velocityRotationsPerSecond;
  private StatusSignal<Voltage>[] appliedVolts;
  private StatusSignal<Current>[] supplyCurrentAmps;
  private StatusSignal<Current>[] torqueCurrentAmps;
  private StatusSignal<Temperature>[] temperatureCelsius;
  private StatusSignal<Double>[] positionSetpointRotations;
  private StatusSignal<Double>[] positionErrorRotations;

  private final Alert disconnectedAlert =
      new Alert("Elevator Talon is disconnected, check CAN bus!", AlertType.ERROR);

  private VoltageOut voltageControlRequest;
  private MotionMagicVoltage positionControlRequest;
  private MotionMagicTorqueCurrentFOC currentControlRequest;

  public ElevatorIOTalonFX() {
    talonFX = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID);
    followTalonFX[0] = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID + 1);
    followTalonFX[1] = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID + 2);
    followTalonFX[2] = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID + 3);

    TalonFXConfiguration config = new TalonFXConfiguration();
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
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO;

    config.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.CONSTRAINTS.maxAcceleration().get();
    config.MotionMagic.MotionMagicCruiseVelocity =
        ElevatorConstants.CONSTRAINTS.cruisingVelocity().get();

    talonFX.getConfigurator().apply(config);
    for (TalonFX follow : followTalonFX) {
      follow.getConfigurator().apply(config);
      follow.setControl(new Follower(talonFX.getDeviceID(), false));
    }

    for (int i = 0; i < positionRotations.length; i++) {
      if (i == 0) {
        positionRotations[i] = talonFX.getPosition();
        velocityRotationsPerSecond[i] = talonFX.getVelocity();
        appliedVolts[i] = talonFX.getMotorVoltage();
        supplyCurrentAmps[i] = talonFX.getSupplyCurrent();
        torqueCurrentAmps[i] = talonFX.getTorqueCurrent();
        temperatureCelsius[i] = talonFX.getDeviceTemp();
        positionSetpointRotations[i] = talonFX.getClosedLoopReference();
        positionErrorRotations[i] = talonFX.getClosedLoopError();
      }
      positionRotations[i] = followTalonFX[i - 1].getPosition();
      velocityRotationsPerSecond[i] = followTalonFX[i - 1].getVelocity();
      appliedVolts[i] = followTalonFX[i - 1].getMotorVoltage();
      supplyCurrentAmps[i] = followTalonFX[i - 1].getSupplyCurrent();
      torqueCurrentAmps[i] = followTalonFX[i - 1].getTorqueCurrent();
      temperatureCelsius[i] = followTalonFX[i - 1].getDeviceTemp();
      positionSetpointRotations[i] = followTalonFX[i - 1].getClosedLoopReference();
      positionErrorRotations[i] = followTalonFX[i - 1].getClosedLoopError();
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        positionRotations[0],
        velocityRotationsPerSecond[0],
        appliedVolts[0],
        supplyCurrentAmps[0],
        torqueCurrentAmps[0],
        temperatureCelsius[0],
        positionSetpointRotations[0],
        positionErrorRotations[0],
        positionRotations[1],
        velocityRotationsPerSecond[1],
        appliedVolts[1],
        supplyCurrentAmps[1],
        torqueCurrentAmps[1],
        temperatureCelsius[1],
        positionSetpointRotations[1],
        positionErrorRotations[1],
        positionRotations[2],
        velocityRotationsPerSecond[2],
        appliedVolts[2],
        supplyCurrentAmps[2],
        torqueCurrentAmps[2],
        temperatureCelsius[2],
        positionSetpointRotations[2],
        positionErrorRotations[2],
        positionRotations[3],
        velocityRotationsPerSecond[3],
        appliedVolts[3],
        supplyCurrentAmps[3],
        torqueCurrentAmps[3],
        temperatureCelsius[3],
        positionSetpointRotations[3],
        positionErrorRotations[3]);

    talonFX.optimizeBusUtilization(50, 1.0);

    voltageControlRequest = new VoltageOut(0.0);
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                positionRotations[0],
                velocityRotationsPerSecond[0],
                appliedVolts[0],
                supplyCurrentAmps[0],
                torqueCurrentAmps[0],
                temperatureCelsius[0],
                positionSetpointRotations[0],
                positionErrorRotations[0],
                positionRotations[1],
                velocityRotationsPerSecond[1],
                appliedVolts[1],
                supplyCurrentAmps[1],
                torqueCurrentAmps[1],
                temperatureCelsius[1],
                positionSetpointRotations[1],
                positionErrorRotations[1],
                positionRotations[2],
                velocityRotationsPerSecond[2],
                appliedVolts[2],
                supplyCurrentAmps[2],
                torqueCurrentAmps[2],
                temperatureCelsius[2],
                positionSetpointRotations[2],
                positionErrorRotations[2],
                positionRotations[3],
                velocityRotationsPerSecond[3],
                appliedVolts[3],
                supplyCurrentAmps[3],
                torqueCurrentAmps[3],
                temperatureCelsius[3],
                positionSetpointRotations[3],
                positionErrorRotations[3])
            .isOK();

    for (int i = 0; i < positionRotations.length; i++) {

      positionSetpointRotations[i].refresh();
      positionErrorRotations[i].refresh();
    }

    disconnectedAlert.set(!connected);

    for (int i = 0; i < positionRotations.length; i++) {
      inputs.positionMeters[i] =
          positionRotations[i].getValueAsDouble()
              * Math.PI
              * ElevatorConstants.DRUM_RADIUS
              * 2
              / ElevatorConstants.ELEVATOR_GEAR_RATIO;
      inputs.velocityMetersPerSecond[i] =
          velocityRotationsPerSecond[i].getValueAsDouble()
              * Math.PI
              * ElevatorConstants.DRUM_RADIUS
              * 2
              / ElevatorConstants.ELEVATOR_GEAR_RATIO;
      inputs.appliedVolts[i] = appliedVolts[i].getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps[i].getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps[i].getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius[i].getValueAsDouble();
      inputs.positionSetpointMeters[i] =
          positionSetpointRotations[i].getValueAsDouble()
              * Math.PI
              * ElevatorConstants.DRUM_RADIUS
              * 2
              / ElevatorConstants.ELEVATOR_GEAR_RATIO;
      inputs.positionErrorMeters[i] =
          positionErrorRotations[i].getValueAsDouble()
              * Math.PI
              * ElevatorConstants.DRUM_RADIUS
              * 2
              / ElevatorConstants.ELEVATOR_GEAR_RATIO;
    }
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setCurrent(double amps) {
    talonFX.setControl(currentControlRequest.withFeedForward(amps));
  }

  @Override
  public void setPosition(double meters) {
    talonFX.setPosition(
        meters
            * ElevatorConstants.ELEVATOR_GEAR_RATIO
            / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS));
  }

  @Override
  public void setPositionGoal(double meters) {
    // talonFX.setControl(
    //     positionControlRequest
    //         .withPosition(
    //             meters
    //                 * ElevatorConstants.ELEVATOR_GEAR_RATIO
    //                 / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS))
    //         .withEnableFOC(true));

    talonFX.setControl(
        currentControlRequest.withPosition(
            meters
                * ElevatorConstants.ELEVATOR_GEAR_RATIO
                / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS)));
  }
}
