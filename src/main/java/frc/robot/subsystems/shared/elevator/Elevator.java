package frc.robot.subsystems.shared.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.util.LoggedTracer;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;

  @Getter private ElevatorPositions position;
  private boolean isClosedLoop;

  public Elevator(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Elevator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));

    position = ElevatorPositions.STOW;
    isClosedLoop = true;
  }

  @Override
  public void periodic() {
    LoggedTracer.reset();
    io.updateInputs(inputs);
    LoggedTracer.record("Update Inputs", "Elevator/Periodic");

    LoggedTracer.reset();
    Logger.processInputs("Elevator", inputs);
    LoggedTracer.record("Process Inputs", "Elevator/Periodic");

    Logger.recordOutput("Elevator/Position", position.name());

    LoggedTracer.reset();
    if (isClosedLoop) {
      io.setPositionGoal(position.getPosition());
    }
    LoggedTracer.record("Set Position Goal", "Elevator/Periodic");
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
            case CORAL_INTAKE:
              this.position = ElevatorPositions.CORAL_INTAKE;
              break;
            case ALGAE_FLOOR_INTAKE:
              this.position = ElevatorPositions.ALGAE_INTAKE;
              break;
            case ALGAE_MID:
              this.position = ElevatorPositions.ALGAE_MID;
              break;
            case ASS_TOP:
              this.position = ElevatorPositions.ASS_TOP;
              break;
            case ASS_BOT:
              this.position = ElevatorPositions.ASS_BOT;
              break;
            case ALGAE_INTAKE_TOP:
              this.position = ElevatorPositions.ALGAE_INTAKE_TOP;
              break;
            case ALGAE_INTAKE_BOTTOM:
              this.position = ElevatorPositions.ALGAE_INTAKE_BOT;
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
            case L4_PLUS:
              this.position = ElevatorPositions.L4_PLUS;
              break;
            case ALGAE_SCORE:
              this.position = ElevatorPositions.ALGAE_SCORE;
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
  public Command setPosition(Supplier<ReefHeight> newPosition) {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          switch (newPosition.get()) {
            case STOW:
              this.position = ElevatorPositions.STOW;
              break;
            case CORAL_INTAKE:
              this.position = ElevatorPositions.CORAL_INTAKE;
              break;
            case ALGAE_FLOOR_INTAKE:
              this.position = ElevatorPositions.ALGAE_INTAKE;
              break;
            case ALGAE_MID:
              this.position = ElevatorPositions.ALGAE_MID;
              break;
            case ASS_TOP:
              this.position = ElevatorPositions.ASS_TOP;
              break;
            case ASS_BOT:
              this.position = ElevatorPositions.ASS_BOT;
              break;
            case ALGAE_INTAKE_TOP:
              this.position = ElevatorPositions.ALGAE_INTAKE_TOP;
              break;
            case ALGAE_INTAKE_BOTTOM:
              this.position = ElevatorPositions.ALGAE_INTAKE_BOT;
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
            case L4_PLUS:
              this.position = ElevatorPositions.L4_PLUS;
              break;
            case ALGAE_SCORE:
              this.position = ElevatorPositions.ALGAE_SCORE;
            default:
              break;
          }
        });
  }

  public ElevatorPositions getPosition(ReefHeight newPosition) {
    switch (newPosition) {
      case STOW:
        return ElevatorPositions.STOW;
      case CORAL_INTAKE:
        return ElevatorPositions.CORAL_INTAKE;
      case ALGAE_FLOOR_INTAKE:
        return ElevatorPositions.ALGAE_INTAKE;
      case ALGAE_MID:
        return ElevatorPositions.ALGAE_MID;
      case ASS_TOP:
        return ElevatorPositions.ASS_TOP;
      case ASS_BOT:
        return ElevatorPositions.ASS_BOT;
      case ALGAE_INTAKE_TOP:
        return ElevatorPositions.ALGAE_INTAKE_TOP;
      case ALGAE_INTAKE_BOTTOM:
        return ElevatorPositions.ALGAE_INTAKE_BOT;
      case L1:
        return ElevatorPositions.L1;
      case L2:
        return ElevatorPositions.L2;
      case L3:
        return ElevatorPositions.L3;
      case L4:
        return ElevatorPositions.L4;
      case L4_PLUS:
        return ElevatorPositions.L4_PLUS;
      case ALGAE_SCORE:
        return ElevatorPositions.ALGAE_SCORE;
      default:
        return ElevatorPositions.STOW;
    }
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
                () -> io.setPosition(ElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS())));
  }

  /**
   * Runs the SysId routine for the elevator.
   *
   * @return A command to run the SysId routine.
   */
  public Command runSysID() {
    return Commands.sequence(
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(() -> atGoal(ElevatorPositions.L4.getPosition() - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> atGoal(ElevatorPositions.STOW.getPosition() + Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kForward)
            .until(() -> atGoal(ElevatorPositions.L4.getPosition() - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kReverse)
            .until(() -> atGoal(ElevatorPositions.STOW.getPosition() + Units.inchesToMeters(12.0))),
        setPosition(() -> ReefHeight.STOW));
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
   * Checks if the elevator is at the goal position within a specified tolerance.
   *
   * @param position The target position in meters.
   * @return true if the current position is within the goal tolerance of the target position, false
   *     otherwise.
   */
  private boolean atGoal(double position) {
    return Math.abs(position - inputs.positionMeters)
        <= ElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }

  /**
   * Checks if the elevator is at the goal position.
   *
   * @return True if the elevator is at the goal position, false otherwise.
   */
  @AutoLogOutput(key = "Elevator/At Goal")
  public boolean atGoal() {
    return Math.abs(inputs.positionGoalMeters - inputs.positionMeters)
        <= ElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }

  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(0.02).andThen(Commands.waitUntil(this::atGoal));
  }
}
