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
    private final TalonFX topTalonFX;
    private final TalonFX bottomTalonFX;

    private StatusSignal<Angle> topPositionRotations;
    private StatusSignal<AngularVelocity> topVelocityRotationsPerSecond;
    private StatusSignal<Voltage> topAppliedVolts;
    private StatusSignal<Current> topSupplyCurrentAmps;
    private StatusSignal<Current> topTorqueCurrentAmps;
    private StatusSignal<Temperature> topTemperatureCelcius;
    private StatusSignal<Double> topPositionSetpointRotations;
    private StatusSignal<Double> topPositionErrorRotations;

    private StatusSignal<Angle> bottomPositionRotations;
    private StatusSignal<AngularVelocity> bottomVelocityRotationsPerSecond;
    private StatusSignal<Voltage> bottomAppliedVolts;
    private StatusSignal<Current> bottomSupplyCurrentAmps;
    private StatusSignal<Current> bottomTorqueCurrentAmps;
    private StatusSignal<Temperature> bottomTemperatureCelcius;
    private StatusSignal<Double> bottomPositionSetpointRotations;
    private StatusSignal<Double> bottomPositionErrorRotations;

    private final Alert topDisconnectedAlert = new Alert("Top Elevator Talon is disconnected, check CAN bus!",
            AlertType.ERROR);
    private final Alert bottomDisconnectedAlert = new Alert("Bottom Elevator Talon is disconnected, check CAN bus!",
            AlertType.ERROR);

    private VoltageOut voltageControlRequest;
    private MotionMagicVoltage positionControlRequest;

    public ElevatorIOTalonFX() {
        topTalonFX = new TalonFX(ElevatorConstants.TOP_ELEVATOR_CAN_ID);
        bottomTalonFX = new TalonFX(ElevatorConstants.BOTTOM_ELEVATOR_CAN_ID);

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

        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.CONSTRAINTS.maxAcceleration().get();
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.GAINS.kV().get();

        topTalonFX.getConfigurator().apply(config);
        bottomTalonFX.getConfigurator().apply(config);

        topPositionRotations = topTalonFX.getPosition();
        topVelocityRotationsPerSecond = topTalonFX.getVelocity();
        topAppliedVolts = topTalonFX.getMotorVoltage();
        topSupplyCurrentAmps = topTalonFX.getSupplyCurrent();
        topTorqueCurrentAmps = topTalonFX.getTorqueCurrent();
        topTemperatureCelcius = topTalonFX.getDeviceTemp();
        topPositionSetpointRotations = topTalonFX.getClosedLoopReference();
        topPositionErrorRotations = topTalonFX.getClosedLoopError();

        bottomPositionRotations = bottomTalonFX.getPosition();
        bottomVelocityRotationsPerSecond = bottomTalonFX.getVelocity();
        bottomAppliedVolts = bottomTalonFX.getMotorVoltage();
        bottomSupplyCurrentAmps = bottomTalonFX.getSupplyCurrent();
        bottomTorqueCurrentAmps = bottomTalonFX.getTorqueCurrent();
        bottomTemperatureCelcius = bottomTalonFX.getDeviceTemp();
        bottomPositionSetpointRotations = bottomTalonFX.getClosedLoopReference();
        bottomPositionErrorRotations = bottomTalonFX.getClosedLoopError();

        BaseStatusSignal.setUpdateFrequencyForAll(100,
                topPositionRotations, topVelocityRotationsPerSecond, topAppliedVolts, topSupplyCurrentAmps,
                topTorqueCurrentAmps, topTemperatureCelcius, topPositionSetpointRotations, topPositionErrorRotations,
                bottomPositionRotations, bottomVelocityRotationsPerSecond, bottomAppliedVolts, bottomSupplyCurrentAmps,
                bottomTorqueCurrentAmps, bottomTemperatureCelcius, bottomPositionSetpointRotations,
                bottomPositionErrorRotations);
        
        topTalonFX.optimizeBusUtilization(50, 1.0);
        bottomTalonFX.optimizeBusUtilization(50, 1.0);

        voltageControlRequest = new VoltageOut(0.0);
        positionControlRequest = new MotionMagicVoltage(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        boolean topConnected = BaseStatusSignal.refreshAll(topPositionRotations, topVelocityRotationsPerSecond,
                topAppliedVolts, topSupplyCurrentAmps, topTorqueCurrentAmps, topTemperatureCelcius,
                topPositionSetpointRotations, topPositionErrorRotations).isOK();
        boolean bottomConnected = BaseStatusSignal.refreshAll(bottomPositionRotations, bottomVelocityRotationsPerSecond,
                bottomAppliedVolts, bottomSupplyCurrentAmps, bottomTorqueCurrentAmps, bottomTemperatureCelcius,
                bottomPositionSetpointRotations, bottomPositionErrorRotations).isOK();

        topPositionSetpointRotations.refresh();
        topPositionErrorRotations.refresh();
        bottomPositionSetpointRotations.refresh();
        bottomPositionErrorRotations.refresh();

        topDisconnectedAlert.set(!topConnected);
        bottomDisconnectedAlert.set(!bottomConnected);

        inputs.topPositionMeters = topPositionRotations.getValueAsDouble()*Math.PI*ElevatorConstants.TOP_DRUM_RADIUS*2/ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO;
        inputs.topVelocityMetersPerSecond = topVelocityRotationsPerSecond.getValueAsDouble()*Math.PI*ElevatorConstants.TOP_DRUM_RADIUS*2/ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO;
        inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
        inputs.topSupplyCurrentAmps = topSupplyCurrentAmps.getValueAsDouble();
        inputs.topTorqueCurrentAmps = topTorqueCurrentAmps.getValueAsDouble();
        inputs.topTemperatureCelcius = topTemperatureCelcius.getValueAsDouble();
        inputs.topPositionSetpointMeters = topPositionSetpointRotations.getValueAsDouble()*Math.PI*ElevatorConstants.TOP_DRUM_RADIUS*2/ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO;
        inputs.topPositionErrorMeters = topPositionErrorRotations.getValueAsDouble()*Math.PI*ElevatorConstants.TOP_DRUM_RADIUS*2/ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO;

        inputs.bottomPositionMeters = bottomPositionRotations.getValueAsDouble()*Math.PI*ElevatorConstants.BOTTOM_DRUM_RADIUS*2/ElevatorConstants.ELEVATOR_BOTTOM_GEAR_RATIO;
        inputs.bottomVelocityMetersPerSecond = bottomVelocityRotationsPerSecond.getValueAsDouble()*Math.PI*ElevatorConstants.BOTTOM_DRUM_RADIUS*2/ElevatorConstants.ELEVATOR_BOTTOM_GEAR_RATIO;
        inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
        inputs.bottomSupplyCurrentAmps = bottomSupplyCurrentAmps.getValueAsDouble();
        inputs.bottomTorqueCurrentAmps = bottomTorqueCurrentAmps.getValueAsDouble();
        inputs.bottomTemperatureCelcius = bottomTemperatureCelcius.getValueAsDouble();
        inputs.bottomPositionSetpointMeters = bottomPositionSetpointRotations.getValueAsDouble()*Math.PI*ElevatorConstants.BOTTOM_DRUM_RADIUS*2/ElevatorConstants.ELEVATOR_BOTTOM_GEAR_RATIO;
        inputs.bottomPositionErrorMeters = bottomPositionErrorRotations.getValueAsDouble()*Math.PI*ElevatorConstants.BOTTOM_DRUM_RADIUS*2/ElevatorConstants.ELEVATOR_BOTTOM_GEAR_RATIO;

    }

    @Override
    public void setBottomVoltage(double volts) {
        bottomTalonFX.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
    }

    @Override
    public void setTopVoltage(double volts) {
        topTalonFX.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(true));
    }

    @Override
    public void setTopPosition(double meters) {
        topTalonFX.setPosition(meters * ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO / (2 * Math.PI * ElevatorConstants.TOP_DRUM_RADIUS));
    }

    @Override
    public void setBottomPosition(double meters) {
        bottomTalonFX.setPosition(meters * ElevatorConstants.ELEVATOR_BOTTOM_GEAR_RATIO / (2 * Math.PI * ElevatorConstants.BOTTOM_DRUM_RADIUS));
    }

    @Override
    public void setTopPositionGoal(double meters) {
        topTalonFX.setControl(positionControlRequest.withPosition(meters* ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO / (2 * Math.PI * ElevatorConstants.TOP_DRUM_RADIUS)).withEnableFOC(true));
    }

    @Override
    public void setBottomPositionGoal(double meters) {
        bottomTalonFX.setControl(positionControlRequest.withPosition(meters * ElevatorConstants.ELEVATOR_BOTTOM_GEAR_RATIO / (2 * Math.PI * ElevatorConstants.BOTTOM_DRUM_RADIUS)).withEnableFOC(true));
    }

}
