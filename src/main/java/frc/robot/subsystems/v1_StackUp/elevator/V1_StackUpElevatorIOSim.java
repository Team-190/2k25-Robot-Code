package frc.robot.subsystems.v1_StackUp.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class V1_StackUpElevatorIOSim implements V1_StackUpElevatorIO {
  private final ElevatorSim sim;

  private final ProfiledPIDController controller;
  private ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  public V1_StackUpElevatorIOSim() {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                V1_StackUpElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
                4,
                V1_StackUpElevatorConstants.DRUM_RADIUS,
                V1_StackUpElevatorConstants.ELEVATOR_GEAR_RATIO),
            V1_StackUpElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
            V1_StackUpElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS(),
            V1_StackUpElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS(),
            true,
            V1_StackUpElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS());

    controller =
        new ProfiledPIDController(
            V1_StackUpElevatorConstants.GAINS.kP().get(),
            0,
            V1_StackUpElevatorConstants.GAINS.kD().get(),
            new Constraints(
                V1_StackUpElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get(),
                V1_StackUpElevatorConstants.CONSTRAINTS
                    .maxAccelerationMetersPerSecondSquared()
                    .get()));

    feedforward =
        new ElevatorFeedforward(
            V1_StackUpElevatorConstants.GAINS.kS().get(),
            V1_StackUpElevatorConstants.GAINS.kG().get(),
            V1_StackUpElevatorConstants.GAINS.kV().get());

    appliedVolts = 0.0;
    isClosedLoop = true;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts =
          controller.calculate(sim.getPositionMeters())
              + feedforward.calculate(controller.getSetpoint().position);
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
    for (int i = 0; i < V1_StackUpElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
      inputs.appliedVolts[i] = appliedVolts;
      inputs.supplyCurrentAmps[i] = sim.getCurrentDrawAmps();
      inputs.torqueCurrentAmps[i] = sim.getCurrentDrawAmps();
      inputs.temperatureCelsius[i] = 0.0;
    }
    inputs.positionGoalMeters = controller.getGoal().position;
    inputs.positionSetpointMeters = controller.getSetpoint().position;
    inputs.positionErrorMeters = controller.getPositionError();
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
    controller.setGoal(position);
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    controller.setPID(kP, 0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    controller.setConstraints(new Constraints(cruisingVelocity, maxAcceleration));
  }
}
