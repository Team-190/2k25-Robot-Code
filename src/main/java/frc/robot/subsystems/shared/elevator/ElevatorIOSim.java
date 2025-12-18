package frc.robot.subsystems.shared.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;

  private final ProfiledPIDController feedback;
  private ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  public ElevatorIOSim() {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                ElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
                4,
                ElevatorConstants.DRUM_RADIUS,
                ElevatorConstants.ELEVATOR_GEAR_RATIO),
            ElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
            ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS(),
            ElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS(),
            true,
            ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS());
    feedback =
        new ProfiledPIDController(
            ElevatorConstants.GAINS.kP().get(),
            0,
            ElevatorConstants.GAINS.kD().get(),
            new Constraints(
                ElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get(),
                ElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().get()));

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.GAINS.kS().get(),
            ElevatorConstants.GAINS.kG().get(),
            ElevatorConstants.GAINS.kV().get());

    appliedVolts = 0.0;
    isClosedLoop = true;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts =
          feedback.calculate(sim.getPositionMeters())
              + feedforward.calculate(feedback.getSetpoint().position);
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    // Position and velocity
    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
    inputs.accelerationMetersPerSecondSquared = 0;

    for (int i = 0; i < ElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
      inputs.appliedVolts[i] = appliedVolts;
      inputs.supplyCurrentAmps[i] = sim.getCurrentDrawAmps();
      inputs.torqueCurrentAmps[i] = sim.getCurrentDrawAmps();
      inputs.temperatureCelsius[i] = 0.0;
    }

    inputs.positionGoalMeters = feedback.getGoal().position;
    inputs.positionSetpointMeters = feedback.getSetpoint().position;
    inputs.positionErrorMeters = feedback.getPositionError();
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setPosition(double position) {
    sim.setState(position, 0);
  }

  @Override
  public void setPositionGoal(double position) {
    isClosedLoop = true;
    feedback.setGoal(position);
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    feedback.setConstraints(new Constraints(cruisingVelocity, maxAcceleration));
  }

  public double getAcceleration(double previousVelocity, double currentVelocity) {
    return (currentVelocity - previousVelocity) / 0.02;
  }
}
