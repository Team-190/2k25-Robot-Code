package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim;
  private double appliedVolts;
  private double positionGoalMeters;

  private ProfiledPIDController controller;
  private ElevatorFeedforward feedforward;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            ElevatorConstants.ELEVATOR_PARAMS.ELEVATOR_MOTOR_CONFIG(),
            ElevatorConstants.ELEVATOR_GEAR_RATIO,
            ElevatorConstants.ELEVATOR_PARAMS.CARRIAGE_MASS_KG(),
            ElevatorConstants.DRUM_RADIUS,
            ElevatorConstants.ELEVATOR_PARAMS.MIN_HEIGHT_METERS(),
            ElevatorConstants.ELEVATOR_PARAMS.MAX_HEIGHT_METERS(),
            true,
            ElevatorConstants.ELEVATOR_PARAMS.MIN_HEIGHT_METERS());
    controller =
        new ProfiledPIDController(
            ElevatorConstants.GAINS.kP().get(),
            0,
            ElevatorConstants.GAINS.kD().get(),
            new Constraints(
                ElevatorConstants.CONSTRAINTS.cruisingVelocityRotsPerSec().get(),
                ElevatorConstants.CONSTRAINTS.maxAccelerationRotsPerSecSq().get()));

    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.GAINS.kS().get(),
            ElevatorConstants.GAINS.kG().get(),
            ElevatorConstants.GAINS.kV().get());

    appliedVolts = 0.0;
    positionGoalMeters = 0.0;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(Constants.LOOP_PERIOD_SECONDS);
    for (int i = 0; i < 4; i++) {
      inputs.appliedVolts[i] = appliedVolts;
      inputs.supplyCurrentAmps[i] = elevatorSim.getCurrentDrawAmps();
      inputs.torqueCurrentAmps[i] = elevatorSim.getCurrentDrawAmps();
    }

    inputs.positionMeters = elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.positionGoalMeters = positionGoalMeters;
    inputs.positionSetpointMeters = controller.getSetpoint().position;
    inputs.positionErrorMeters = controller.getPositionError();
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
    controller.setConstraints(
        new Constraints(cruisingVelocity, maxAcceleration));
  }
}
