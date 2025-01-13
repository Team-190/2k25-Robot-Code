package frc.robot.subsystems.v1_gamma.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.v1_gamma.elevator.ElevatorConstants.ElevatorPositions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private ElevatorConstants.ElevatorPositions position;

  public Elevator(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();

    position = ElevatorPositions.STOW;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/Position", position.name());
    io.setBottomPositionGoal(position.getBottomPosition());
    io.setTopPositionGoal(position.getTopPosition());
  }

  public Command setPosition(ElevatorConstants.ElevatorPositions position) {
    return runOnce(() -> this.position = position);
  }

  public Command resetPosition() {
    return runOnce(() -> this.position = ElevatorPositions.STOW)
        .andThen(
            runOnce(
                () -> {
                  io.setBottomPosition(ElevatorConstants.BOTTOM_MIN_HEIGHT_METERS);
                  io.setTopPosition(ElevatorConstants.TOP_MIN_HEIGHT_METERS);
                }));
  }

  public ElevatorConstants.ElevatorPositions getPosition() {
    return position;
  }
}
