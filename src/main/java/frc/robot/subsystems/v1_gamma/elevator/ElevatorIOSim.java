package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim topElevatorSim;
    private ElevatorSim bottomElevatorSim;

    private double topPositionMeters;
    private double topVelocityMetersPerSecond;
    private double topAppliedVolts;
    private double topSupplyCurrentAmps;
    private double topTorqueCurrentAmps;
    private double topTemperatureCelcius;

    private double bottomPositionMeters;
    private double bottomVelocityMetersPerSecond;
    private double bottomAppliedVolts;
    private double bottomSupplyCurrentAmps;
    private double bottomTorqueCurrentAmps;
    private double bottomTemperatureCelcius;


    public ElevatorIOSim() {
        topElevatorSim = new ElevatorSim(
            ElevatorConstants.ELEVATOR_MOTOR_CONFIG,
            ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO,
            ElevatorConstants.TOP_CARRIAGE_MASS_KG,
            ElevatorConstants.TOP_DRUM_RADIUS,
            ElevatorConstants.TOP_MIN_HEIGHT_METERS,
            ElevatorConstants.TOP_MAX_HEIGHT_METERS,
            true,
            ElevatorConstants.TOP_MIN_HEIGHT_METERS);

        bottomElevatorSim = new ElevatorSim(
            ElevatorConstants.ELEVATOR_MOTOR_CONFIG,
            ElevatorConstants.ELEVATOR_BOTTOM_GEAR_RATIO,
            ElevatorConstants.BOTTOM_CARRIAGE_MASS_KG,
            ElevatorConstants.BOTTOM_DRUM_RADIUS,
            ElevatorConstants.BOTTOM_MIN_HEIGHT_METERS,
            ElevatorConstants.BOTTOM_MAX_HEIGHT_METERS,
            true,
            ElevatorConstants.BOTTOM_MIN_HEIGHT_METERS
        );

        topPositionMeters = 0.0;
        topVelocityMetersPerSecond = 0.0;
        topAppliedVolts = 0.0;
        topSupplyCurrentAmps = 0.0;
        topTorqueCurrentAmps = 0.0;
        topTemperatureCelcius = 0.0;

        bottomPositionMeters = 0.0;
        bottomVelocityMetersPerSecond = 0.0;
        bottomAppliedVolts = 0.0;
        bottomSupplyCurrentAmps = 0.0;
        bottomTorqueCurrentAmps = 0.0;
        bottomTemperatureCelcius = 0.0;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.topPositionMeters = topElevatorSim.getPositionMeters();
        inputs.topVelocityMetersPerSecond = topElevatorSim.getVelocityMetersPerSecond();
        inputs.topAppliedVolts = topAppliedVolts;
        inputs.topSupplyCurrentAmps = topElevatorSim.getCurrentDrawAmps();
        inputs.topTorqueCurrentAmps = topElevatorSim.getCurrentDrawAmps();
        inputs.topTemperatureCelcius = topTemperatureCelcius;

        inputs.bottomPositionMeters = bottomElevatorSim.getPositionMeters();
        inputs.bottomVelocityMetersPerSecond = bottomElevatorSim.getVelocityMetersPerSecond();
        inputs.bottomAppliedVolts = bottomAppliedVolts;
        inputs.bottomSupplyCurrentAmps = bottomElevatorSim.getCurrentDrawAmps();
        inputs.bottomTorqueCurrentAmps = bottomElevatorSim.getCurrentDrawAmps();
        inputs.bottomTemperatureCelcius = bottomTemperatureCelcius;
        

    }
    @Override
    public void setBottomVoltage(double volts) {
        bottomAppliedVolts = volts;
        bottomElevatorSim.setInputVoltage(volts);

    }
    @Override
    public void setTopVoltage(double volts) {
        topAppliedVolts = volts;
        topElevatorSim.setInputVoltage(volts);
    }
}
