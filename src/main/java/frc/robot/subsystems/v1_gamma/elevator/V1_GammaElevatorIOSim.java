package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class V1_GammaElevatorIOSim implements V1_GammaElevatorIO {
  private ElevatorSim elevatorSim;
  private double appliedVolts;
  private double positionGoalMeters;

  private ProfiledPIDController controller;
  private ElevatorFeedforward feedforward;

  public V1_GammaElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
            V1_GammaElevatorConstants.ELEVATOR_GEAR_RATIO,
            V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.CARRIAGE_MASS_KG(),
            V1_GammaElevatorConstants.DRUM_RADIUS,
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
                V1_GammaElevatorConstants.CONSTRAINTS.cruisingVelocityRotsPerSec().get(),
                V1_GammaElevatorConstants.CONSTRAINTS.maxAccelerationRotsPerSecSq().get()));

    feedforward =
        new ElevatorFeedforward(
            V1_GammaElevatorConstants.GAINS.kS().get(),
            V1_GammaElevatorConstants.GAINS.kG().get(),
            V1_GammaElevatorConstants.GAINS.kV().get());

    appliedVolts = 0.0;
    positionGoalMeters = 0.0;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(Constants.LOOP_PERIOD_SECONDS);

    inputs.positionMeters = elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.positionGoalMeters = positionGoalMeters;
    inputs.positionSetpointMeters = controller.getSetpoint().position;
    inputs.positionErrorMeters = controller.getPositionError();
    for (int i = 0; i < 4; i++) {
      inputs.appliedVolts[i] = appliedVolts;
      inputs.supplyCurrentAmps[i] = elevatorSim.getCurrentDrawAmps();
      inputs.torqueCurrentAmps[i] = elevatorSim.getCurrentDrawAmps();
      inputs.temperatureCelsius[i] = 0.0;
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void setPosition(double position) {
    elevatorSim.setState(position, 0);
  }

  @Override
  public void setPositionGoal(double position) {
    positionGoalMeters = position;
    appliedVolts =
        MathUtil.clamp(
            controller.calculate(position)
                + feedforward.calculate(controller.getSetpoint().position),
            -12,
            12);
  }

  @Override
  public void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    controller.setPID(kP, 0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV);
  }

  @Override
  public void setConstraints(double maxAcceleration, double cruisingVelocity) {
    controller.setConstraints(new Constraints(cruisingVelocity, maxAcceleration));
  }
}
