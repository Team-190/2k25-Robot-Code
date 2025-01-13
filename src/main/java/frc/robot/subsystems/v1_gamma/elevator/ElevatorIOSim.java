package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim topElevatorSim;
  private ElevatorSim bottomElevatorSim;

  private double topAppliedVolts;

  private double bottomAppliedVolts;

  private ProfiledPIDController topController;
  private ProfiledPIDController bottomController;
  private SimpleMotorFeedforward topFeedforward;
  private SimpleMotorFeedforward bottomFeedforward;

  public ElevatorIOSim() {
    topElevatorSim =
        new ElevatorSim(
            ElevatorConstants.ELEVATOR_MOTOR_CONFIG,
            ElevatorConstants.ELEVATOR_TOP_GEAR_RATIO,
            ElevatorConstants.TOP_CARRIAGE_MASS_KG,
            ElevatorConstants.TOP_DRUM_RADIUS,
            ElevatorConstants.TOP_MIN_HEIGHT_METERS,
            ElevatorConstants.TOP_MAX_HEIGHT_METERS,
            true,
            ElevatorConstants.TOP_MIN_HEIGHT_METERS);

    bottomElevatorSim =
        new ElevatorSim(
            ElevatorConstants.ELEVATOR_MOTOR_CONFIG,
            ElevatorConstants.ELEVATOR_BOTTOM_GEAR_RATIO,
            ElevatorConstants.BOTTOM_CARRIAGE_MASS_KG,
            ElevatorConstants.BOTTOM_DRUM_RADIUS,
            ElevatorConstants.BOTTOM_MIN_HEIGHT_METERS,
            ElevatorConstants.BOTTOM_MAX_HEIGHT_METERS,
            true,
            ElevatorConstants.BOTTOM_MIN_HEIGHT_METERS);

    topController =
        new ProfiledPIDController(
            ElevatorConstants.GAINS.kP().get(),
            0,
            ElevatorConstants.GAINS.kD().get(),
            new Constraints(
                ElevatorConstants.CONSTRAINTS.maxAcceleration().get(), Double.POSITIVE_INFINITY));
    bottomController =
        new ProfiledPIDController(
            ElevatorConstants.GAINS.kP().get(),
            0,
            ElevatorConstants.GAINS.kD().get(),
            new Constraints(
                ElevatorConstants.CONSTRAINTS.maxAcceleration().get(), Double.POSITIVE_INFINITY));

    topFeedforward =
        new SimpleMotorFeedforward(
            ElevatorConstants.GAINS.kS().get(),
            ElevatorConstants.GAINS.kV().get(),
            ElevatorConstants.GAINS.kA().get());

    bottomFeedforward =
        new SimpleMotorFeedforward(
            ElevatorConstants.GAINS.kS().get(),
            ElevatorConstants.GAINS.kV().get(),
            ElevatorConstants.GAINS.kA().get());

    topAppliedVolts = 0.0;
    bottomAppliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.topPositionMeters = topElevatorSim.getPositionMeters();
    inputs.topVelocityMetersPerSecond = topElevatorSim.getVelocityMetersPerSecond();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topSupplyCurrentAmps = topElevatorSim.getCurrentDrawAmps();
    inputs.topTorqueCurrentAmps = topElevatorSim.getCurrentDrawAmps();

    inputs.topPositionSetpointMeters = topController.getSetpoint().position;
    inputs.topPositionErrorMeters = topController.getPositionError();

    inputs.bottomPositionMeters = bottomElevatorSim.getPositionMeters();
    inputs.bottomVelocityMetersPerSecond = bottomElevatorSim.getVelocityMetersPerSecond();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomSupplyCurrentAmps = bottomElevatorSim.getCurrentDrawAmps();
    inputs.bottomTorqueCurrentAmps = bottomElevatorSim.getCurrentDrawAmps();

    inputs.bottomPositionSetpointMeters = bottomController.getSetpoint().position;
    inputs.bottomPositionErrorMeters = bottomController.getPositionError();
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

  @Override
  public void setTopPositionGoal(double position) {
    topElevatorSim.setInput(
        topController.calculate(position)
            + topFeedforward.calculate(topController.getSetpoint().position));
  }

  @Override
  public void setBottomPositionGoal(double position) {
    bottomElevatorSim.setInput(
        bottomController.calculate(position)
            + bottomFeedforward.calculate(bottomController.getSetpoint().position));
  }
}
