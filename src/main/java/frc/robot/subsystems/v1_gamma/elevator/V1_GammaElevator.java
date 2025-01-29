package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.KSCharacterization;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorConstants.ElevatorPositions;
import org.littletonrobotics.junction.Logger;

public class V1_GammaElevator extends SubsystemBase {
  private final V1_GammaElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private ElevatorPositions position;
  private final KSCharacterization characterizationRoutine;

  private boolean isClosedLoop;

  public V1_GammaElevator(V1_GammaElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();

    position = ElevatorPositions.STOW;

    isClosedLoop = true;
    characterizationRoutine =
        new KSCharacterization(this, io::setCurrent, this::getFFCharacterizationVelocity);
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
  public Command setPosition(V1_GammaElevatorConstants.ElevatorPositions position) {
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
            runOnce(
                () ->
                    io.setPosition(
                        V1_GammaElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS())));
  }

  /**
   * Runs the system identification routine.
   *
   * @return A command that runs the system identification routine.
   */
  public Command runCharacterization() {
    return Commands.sequence(runOnce(() -> isClosedLoop = false), characterizationRoutine);
  }

  /**
   * Gets the current position of the elevator.
   *
   * @return The current elevator position.
   */
  public V1_GammaElevatorConstants.ElevatorPositions getPosition() {
    return position;
  }

  /**
   * Gets the feedforward characterization velocity.
   *
   * @return The feedforward characterization velocity.
   */
  public double getFFCharacterizationVelocity() {
    return inputs.velocityMetersPerSecond
        * V1_GammaElevatorConstants.ELEVATOR_GEAR_RATIO
        / (2 * Math.PI * V1_GammaElevatorConstants.DRUM_RADIUS);
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
        < V1_GammaElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }
}
