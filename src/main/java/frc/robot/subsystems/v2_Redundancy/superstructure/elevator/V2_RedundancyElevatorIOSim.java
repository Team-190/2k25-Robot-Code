package frc.robot.subsystems.v2_Redundancy.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class V2_RedundancyElevatorIOSim implements V2_RedundancyElevatorIO {
  private final ElevatorSim sim;

  private final ProfiledPIDController feedback;
  private ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  public V2_RedundancyElevatorIOSim() {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
                4,
                V2_RedundancyElevatorConstants.DRUM_RADIUS,
                V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO),
            V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
            V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS(),
            V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS(),
            true,
            V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS());

    feedback =
        new ProfiledPIDController(
            V2_RedundancyElevatorConstants.GAINS.kP().get(),
            0,
            V2_RedundancyElevatorConstants.GAINS.kD().get(),
            new Constraints(
                V2_RedundancyElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get(),
                V2_RedundancyElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().get()));

    feedforward =
        new ElevatorFeedforward(
            V2_RedundancyElevatorConstants.GAINS.kS().get(),
            V2_RedundancyElevatorConstants.GAINS.kG().get(),
            V2_RedundancyElevatorConstants.GAINS.kV().get());

    appliedVolts = 0.0;
    isClosedLoop = true;
  }

  @Override
  public void updateInputs(V2_RedundancyElevatorIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts =
          feedback.calculate(sim.getPositionMeters())
              + feedforward.calculate(feedback.getSetpoint().position);
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
    for (int i = 0; i < V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
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
}
