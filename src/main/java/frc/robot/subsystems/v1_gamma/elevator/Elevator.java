package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.KSCharacterization;
import frc.robot.subsystems.v1_gamma.elevator.ElevatorConstants.ElevatorPositions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private ElevatorConstants.ElevatorPositions position;
  private boolean isClosedLoop;
  private final KSCharacterization ksRoutine;

  public Elevator(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();

    position = ElevatorPositions.STOW;

    isClosedLoop = true;
    ksRoutine = new KSCharacterization(this, io::setCurrent, this::getFFCharacterizationVelocity);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/Position", position.name());

    if (isClosedLoop) {
      io.setPositionGoal(position.getPosition());
    }
  }

  /**
   * Sets the position of the elevator.
   *
   * @param position The desired elevator position.
   * @return A command that sets the elevator position.
   */
  public Command setPosition(ElevatorConstants.ElevatorPositions position) {
    return runOnce(
        () -> {
          isClosedLoop = true;
          this.position = position;
        });
  }

  /**
   * Resets the elevator position to the stow position.
   *
   * @return A command that resets the elevator position.
   */
  public Command resetPosition() {
    return runOnce(() -> this.position = ElevatorPositions.STOW)
        .andThen(
            runOnce(() -> io.setPosition(ElevatorConstants.ELEVATOR_PARAMS.MIN_HEIGHT_METERS())));
  }

  /**
   * Runs the system identification routine.
   *
   * @return A command that runs the system identification routine.
   */
  public Command runSysId() {
    return Commands.sequence(runOnce(() -> isClosedLoop = false), ksRoutine);
  }

  /**
   * Gets the current position of the elevator.
   *
   * @return The current elevator position.
   */
  public ElevatorConstants.ElevatorPositions getPosition() {
    return position;
  }

  /**
   * Gets the feedforward characterization velocity.
   *
   * @return The feedforward characterization velocity.
   */
  public double getFFCharacterizationVelocity() {
    return inputs.positionMeters
        * ElevatorConstants.ELEVATOR_GEAR_RATIO
        / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
  }

  /**
   * Sets the control gains for the elevator.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   * @param kG The gravity gain.
   */
  public void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.setGains(kP, kD, kS, kV, kA, kG);
  }

  /**
   * Sets the motion constraints for the elevator.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  public void setConstraints(double maxAcceleration, double cruisingVelocity) {
    io.setConstraints(maxAcceleration, cruisingVelocity);
  }

  /**
   * Checks if the elevator is at the goal position.
   *
   * @return True if the elevator is at the goal position, false otherwise.
   */
  public boolean atGoal() {
    return Math.abs(inputs.positionErrorMeters)
        < ElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }
}
