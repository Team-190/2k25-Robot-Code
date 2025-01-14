package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;
  private double appliedVolts;

  private ProfiledPIDController controller;
  private SimpleMotorFeedforward feedforward;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            ElevatorConstants.ELEVATOR_SIM_PARAMS.ELEVATOR_MOTOR_CONFIG(),
            ElevatorConstants.ELEVATOR_GEAR_RATIO,
            ElevatorConstants.ELEVATOR_SIM_PARAMS.CARRIAGE_MASS_KG(),
            ElevatorConstants.DRUM_RADIUS,
            ElevatorConstants.ELEVATOR_SIM_PARAMS.MIN_HEIGHT_METERS(),
            ElevatorConstants.ELEVATOR_SIM_PARAMS.MAX_HEIGHT_METERS(),
            true,
            ElevatorConstants.ELEVATOR_SIM_PARAMS.MIN_HEIGHT_METERS());
    controller =
        new ProfiledPIDController(
            ElevatorConstants.GAINS.kP().get(),
            0,
            ElevatorConstants.GAINS.kD().get(),
            new Constraints(
                ElevatorConstants.CONSTRAINTS.maxAcceleration().get(), Double.POSITIVE_INFINITY));

    feedforward =
        new SimpleMotorFeedforward(
            ElevatorConstants.GAINS.kS().get(),
            ElevatorConstants.GAINS.kV().get(),
            ElevatorConstants.GAINS.kA().get());

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    for (int i = 0; i < 4; i++) {
      inputs.positionMeters[i] = elevatorSim.getPositionMeters();
      inputs.velocityMetersPerSecond[i] = elevatorSim.getVelocityMetersPerSecond();
      inputs.appliedVolts[i] = appliedVolts;
      inputs.supplyCurrentAmps[i] = elevatorSim.getCurrentDrawAmps();
      inputs.torqueCurrentAmps[i] = elevatorSim.getCurrentDrawAmps();

      inputs.positionSetpointMeters[i] = controller.getSetpoint().position;
      inputs.positionErrorMeters[i] = controller.getPositionError();
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(double position) {
    elevatorSim.setState(position, 0);
  }

  @Override
  public void setPositionGoal(double position) {
    elevatorSim.setInput(
        controller.calculate(position) + feedforward.calculate(controller.getSetpoint().position));
  }
}
