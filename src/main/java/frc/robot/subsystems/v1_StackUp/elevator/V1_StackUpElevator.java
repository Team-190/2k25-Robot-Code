package frc.robot.subsystems.v1_StackUp.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.RobotState;
import frc.robot.subsystems.v1_StackUp.elevator.V1_StackUpElevatorConstants.ElevatorPositions;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_StackUpElevator extends SubsystemBase {
  private final V1_StackUpElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;

  @Getter private ElevatorPositions position;
  private boolean isClosedLoop;

  public V1_StackUpElevator(V1_StackUpElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(1),
                Seconds.of(0.5),
                (state) -> Logger.recordOutput("Funnel/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));

    position = ElevatorPositions.STOW;
    isClosedLoop = true;
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
   * @return A command that sets the elevator position.
   */
  public Command setPosition() {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          switch (RobotState.getOIData().currentReefHeight()) {
            case STOW:
              this.position = ElevatorPositions.STOW;
              break;
            case INTAKE:
              this.position = ElevatorPositions.INTAKE;
              break;
            case L1:
              this.position = ElevatorPositions.L1;
              break;
            case L2:
              this.position = ElevatorPositions.L2;
              break;
            case L3:
              this.position = ElevatorPositions.L3;
              break;
            case L4:
              this.position = ElevatorPositions.L4;
              break;
            case TOP_ALGAE:
              this.position = ElevatorPositions.TOP_ALGAE;
              break;
            case BOT_ALGAE:
              this.position = ElevatorPositions.BOT_ALGAE;
              break;
            default:
              break;
          }
        });
  }

  /**
   * Sets the position of the elevator.
   *
   * @param positionRadians The desired elevator position.
   * @return A command that sets the elevator position.
   */
  public Command setPosition(ReefHeight newPosition) {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          switch (newPosition) {
            case STOW:
              this.position = ElevatorPositions.STOW;
              break;
            case INTAKE:
              this.position = ElevatorPositions.INTAKE;
              break;
            case L1:
              this.position = ElevatorPositions.L1;
              break;
            case L2:
              this.position = ElevatorPositions.L2;
              break;
            case L3:
              this.position = ElevatorPositions.L3;
              break;
            case L4:
              this.position = ElevatorPositions.L4;
              break;
            case TOP_ALGAE:
              this.position = ElevatorPositions.TOP_ALGAE;
              break;
            case BOT_ALGAE:
              this.position = ElevatorPositions.BOT_ALGAE;
              break;
            default:
              break;
          }
        });
  }

  public Command setVoltage(double volts) {
    return runEnd(
        () -> {
          isClosedLoop = false;
          io.setVoltage(volts);
        },
        () -> io.setVoltage(0.0));
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
                        V1_StackUpElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS())));
  }

  /**
   * Runs the SysId routine for the elevator.
   *
   * @return A command to run the SysId routine.
   */
  public Command sysIdRoutine() {
    return Commands.sequence(
        runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(2),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(2),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(2),
        characterizationRoutine.dynamic(Direction.kReverse));
  }

  /**
   * Gets the current position of the elevator.
   *
   * @return The current elevator position.
   */
  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  /**
   * Gets the feedforward characterization velocity.
   *
   * @return The feedforward characterization velocity.
   */
  public double getFFCharacterizationVelocity() {
    return inputs.velocityMetersPerSecond
        * V1_StackUpElevatorConstants.ELEVATOR_GEAR_RATIO
        / (2 * Math.PI * V1_StackUpElevatorConstants.DRUM_RADIUS);
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
    io.updateGains(kP, kD, kS, kV, kA, kG);
  }

  /**
   * Sets the motion constraints for the elevator.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param cruisingVelocity The cruising velocity.
   */
  public void setConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateConstraints(maxAcceleration, cruisingVelocity);
  }

  /**
   * Checks if the elevator is at the goal position.
   *
   * @return True if the elevator is at the goal position, false otherwise.
   */
  @AutoLogOutput(key = "Elevator/At Goal")
  public boolean atGoal() {
    return Math.abs(inputs.positionGoalMeters - inputs.positionMeters)
        <= V1_StackUpElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }
}
