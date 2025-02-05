package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class V1_GammaElevatorIOSim implements V1_GammaElevatorIO {
  private ElevatorSim sim;

  private ProfiledPIDController controller;
  private ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  public V1_GammaElevatorIOSim() {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
                4,
                V1_GammaElevatorConstants.DRUM_RADIUS,
                V1_GammaElevatorConstants.ELEVATOR_GEAR_RATIO),
            V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
            V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS(),
            V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS(),
            true,
            V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS());

    controller =
        new ProfiledPIDController(
            V1_GammaElevatorConstants.GAINS.kP().get(),
            0,
            V1_GammaElevatorConstants.GAINS.kD().get(),
            new Constraints(
                V1_GammaElevatorConstants.CONSTRAINTS.cruisingVelocityRadiansPerSecond().get(),
                V1_GammaElevatorConstants.CONSTRAINTS
                    .maxAccelerationRadiansPerSecondSquared()
                    .get()));

    feedforward =
        new ElevatorFeedforward(
            V1_GammaElevatorConstants.GAINS.kS().get(),
            V1_GammaElevatorConstants.GAINS.kG().get(),
            V1_GammaElevatorConstants.GAINS.kV().get());

    appliedVolts = 0.0;
    isClosedLoop = true;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts =
          MathUtil.clamp(
              controller.calculate(sim.getPositionMeters())
                  + feedforward.calculate(controller.getSetpoint().position),
              -12.0,
              12.0);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
    for (int i = 0; i < V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
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
  public void setPosition(double positionMeters) {
    sim.setState(positionMeters, 0);
  }

  @Override
  public void setPositionGoal(double positionMeters) {
    isClosedLoop = true;
    controller.setGoal(positionMeters); 
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
