package frc.robot.subsystems.v1_gamma.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> torqueCurrentAmps;
  private StatusSignal<Temperature> temperatureCelsius;
  private StatusSignal<Double> positionSetpointRotations;
  private StatusSignal<Double> positionErrorRotations;

  private final Alert disconnectedAlert =
      new Alert("Elevator Talon is disconnected, check CAN bus!", AlertType.ERROR);

  private VoltageOut voltageControlRequest;
  private MotionMagicVoltage positionControlRequest;

  public ElevatorIOTalonFX() {
    talonFX = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ElevatorConstants.GAINS.kP().get();
    config.Slot0.kD = ElevatorConstants.GAINS.kD().get();
    config.Slot0.kS = ElevatorConstants.GAINS.kS().get();
    config.Slot0.kV = ElevatorConstants.GAINS.kV().get();
    config.Slot0.kA = ElevatorConstants.GAINS.kA().get();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO;

    config.MotionMagic.MotionMagicAcceleration =
        ElevatorConstants.CONSTRAINTS.maxAcceleration().get();
    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.GAINS.kV().get();

    talonFX.getConfigurator().apply(config);

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
    appliedVolts = talonFX.getMotorVoltage();
    supplyCurrentAmps = talonFX.getSupplyCurrent();
    torqueCurrentAmps = talonFX.getTorqueCurrent();
    temperatureCelsius = talonFX.getDeviceTemp();
    positionSetpointRotations = talonFX.getClosedLoopReference();
    positionErrorRotations = talonFX.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        positionRotations,
        velocityRotationsPerSecond,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temperatureCelsius,
        positionSetpointRotations,
        positionErrorRotations);

    talonFX.optimizeBusUtilization(50, 1.0);

    voltageControlRequest = new VoltageOut(0.0);
    positionControlRequest = new MotionMagicVoltage(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                positionRotations,
                velocityRotationsPerSecond,
                appliedVolts,
                supplyCurrentAmps,
                torqueCurrentAmps,
                temperatureCelsius,
                positionSetpointRotations,
                positionErrorRotations)
            .isOK();

    positionSetpointRotations.refresh();
    positionErrorRotations.refresh();

    disconnectedAlert.set(!connected);

    inputs.positionMeters =
        positionRotations.getValueAsDouble()
            * Math.PI
            * ElevatorConstants.DRUM_RADIUS
            * 2
            / ElevatorConstants.ELEVATOR_GEAR_RATIO;
    inputs.velocityMetersPerSecond =
        velocityRotationsPerSecond.getValueAsDouble()
            * Math.PI
            * ElevatorConstants.DRUM_RADIUS
            * 2
            / ElevatorConstants.ELEVATOR_GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
    inputs.positionSetpointMeters =
        positionSetpointRotations.getValueAsDouble()
            * Math.PI
            * ElevatorConstants.DRUM_RADIUS
            * 2
            / ElevatorConstants.ELEVATOR_GEAR_RATIO;
    inputs.positionErrorMeters =
        positionErrorRotations.getValueAsDouble()
            * Math.PI
            * ElevatorConstants.DRUM_RADIUS
            * 2
            / ElevatorConstants.ELEVATOR_GEAR_RATIO;
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
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
    talonFX.setControl(
        positionControlRequest
            .withPosition(
                meters
                    * ElevatorConstants.ELEVATOR_GEAR_RATIO
                    / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS))
            .withEnableFOC(true));
  }
}
