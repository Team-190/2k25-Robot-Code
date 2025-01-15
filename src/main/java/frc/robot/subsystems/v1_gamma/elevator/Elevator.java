package frc.robot.subsystems.v1_gamma.elevator;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v1_gamma.elevator.ElevatorConstants.ElevatorPositions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private ElevatorConstants.ElevatorPositions position;
  private boolean isClosedLoop;
  private final SysIdRoutine sysIdRoutine;

  public Elevator(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();

    position = ElevatorPositions.STOW;

    isClosedLoop = true;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10.0),
                (state) -> Logger.recordOutput("Elevator", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setCurrent(volts.in(Volts)), null, this));
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
    return runOnce(() -> {
      isClosedLoop = true;
      this.position = position;
    });
  }

  public Command resetPosition() {
    return runOnce(() -> this.position = ElevatorPositions.STOW)
        .andThen(
            runOnce(
                () -> io.setPosition(ElevatorConstants.ELEVATOR_SIM_PARAMS.MIN_HEIGHT_METERS())));
  }

  public Command runSysId() {
    return Commands.sequence(
      runOnce(()->isClosedLoop = false),
      sysIdRoutine.quasistatic(Direction.kForward),
      Commands.waitSeconds(2),
      sysIdRoutine.quasistatic(Direction.kReverse),
      Commands.waitSeconds(2),
      sysIdRoutine.dynamic(Direction.kForward),
      Commands.waitSeconds(2),
      sysIdRoutine.dynamic(Direction.kReverse)
    );
  }

  public ElevatorConstants.ElevatorPositions getPosition() {
    return position;
  }
}
