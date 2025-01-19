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

  public Command setPosition(ElevatorConstants.ElevatorPositions position) {
    return runOnce(
        () -> {
          isClosedLoop = true;
          this.position = position;
        });
  }

  public Command resetPosition() {
    return runOnce(() -> this.position = ElevatorPositions.STOW)
        .andThen(
            runOnce(
                () -> io.setPosition(ElevatorConstants.ELEVATOR_PARAMS.MIN_HEIGHT_METERS())));
  }

  public Command runSysId() {
    return Commands.sequence(runOnce(() -> isClosedLoop = false), ksRoutine);
  }

  public ElevatorConstants.ElevatorPositions getPosition() {
    return position;
  }

  public double getFFCharacterizationVelocity() {
    return inputs.positionMeters
        * ElevatorConstants.ELEVATOR_GEAR_RATIO
        / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS);
  }

  public void setGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.setGains(kP, kD, kS, kV, kA, kG);
  }

  public void setConstraints(double maxAcceleration, double cruisingVelocity) {
    io.setConstraints(maxAcceleration, cruisingVelocity);
  }
}
