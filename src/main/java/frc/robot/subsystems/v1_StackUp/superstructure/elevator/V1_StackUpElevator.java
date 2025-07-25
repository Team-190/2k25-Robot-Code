package frc.robot.subsystems.v1_StackUp.superstructure.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotStateLL;
import frc.robot.subsystems.v1_StackUp.superstructure.elevator.V1_StackUpElevatorConstants.V1_StackUpElevatorPositions;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_StackUpElevator extends SubsystemBase {
  private final V1_StackUpElevatorIO io;
  private final V1_StackUpElevatorIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;

  @Getter private V1_StackUpElevatorPositions position;
  private boolean isClosedLoop;

  public V1_StackUpElevator(V1_StackUpElevatorIO io) {
    this.io = io;
    inputs = new V1_StackUpElevatorIOInputsAutoLogged();

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Elevator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, this));

    position = V1_StackUpElevatorPositions.STOW;
    isClosedLoop = true;
  }

  @Override
  public void periodic() {
    ExternalLoggedTracer.reset();
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Elevator/Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Elevator", inputs);
    InternalLoggedTracer.record("Process Inputs", "Elevator/Periodic");

    Logger.recordOutput("Elevator/Position", position.name());

    InternalLoggedTracer.reset();
    if (isClosedLoop) {
      io.setPositionGoal(position.getPosition());
    }
    InternalLoggedTracer.record("Set Position Goal", "Elevator/Periodic");
    ExternalLoggedTracer.record("Elevator Total", "Elevator/Periodic");
  }

  /**
   * Sets the position of the elevator.
   *
   * @return A command that sets the elevator position.
   */
  public Command setPosition() {
    return this.runOnce(
        () -> {
          isClosedLoop = true;
          switch (RobotStateLL.getOIData().currentReefHeight()) {
            case STOW:
              this.position = V1_StackUpElevatorPositions.STOW;
              break;
            case CORAL_INTAKE:
              this.position = V1_StackUpElevatorPositions.CORAL_INTAKE;
              break;
            case ASS_TOP:
              this.position = V1_StackUpElevatorPositions.ASS_TOP;
              break;
            case ASS_BOT:
              this.position = V1_StackUpElevatorPositions.ASS_BOT;
              break;
            case L1:
              this.position = V1_StackUpElevatorPositions.L1;
              break;
            case L2:
              this.position = V1_StackUpElevatorPositions.L2;
              break;
            case L3:
              this.position = V1_StackUpElevatorPositions.L3;
              break;
            case L4:
              this.position = V1_StackUpElevatorPositions.L4;
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
  public Command setPosition(Supplier<ReefState> newPosition) {
    return this.runOnce(
        () -> {
          isClosedLoop = true;
          switch (newPosition.get()) {
            case STOW:
              this.position = V1_StackUpElevatorPositions.STOW;
              break;
            case CORAL_INTAKE:
              this.position = V1_StackUpElevatorPositions.CORAL_INTAKE;
              break;
            case ASS_TOP:
              this.position = V1_StackUpElevatorPositions.ASS_TOP;
              break;
            case ASS_BOT:
              this.position = V1_StackUpElevatorPositions.ASS_BOT;
              break;
            case L1:
              this.position = V1_StackUpElevatorPositions.L1;
              break;
            case L2:
              this.position = V1_StackUpElevatorPositions.L2;
              break;
            case L3:
              this.position = V1_StackUpElevatorPositions.L3;
              break;
            case L4:
              this.position = V1_StackUpElevatorPositions.L4;
              break;
            default:
              break;
          }
        });
  }

  public V1_StackUpElevatorPositions getPosition(ReefState newPosition) {
    switch (newPosition) {
      case STOW:
        return V1_StackUpElevatorPositions.STOW;
      case CORAL_INTAKE:
        return V1_StackUpElevatorPositions.CORAL_INTAKE;
      case ASS_TOP:
        return V1_StackUpElevatorPositions.ASS_TOP;
      case ASS_BOT:
        return V1_StackUpElevatorPositions.ASS_BOT;
      case L1:
        return V1_StackUpElevatorPositions.L1;
      case L2:
        return V1_StackUpElevatorPositions.L2;
      case L3:
        return V1_StackUpElevatorPositions.L3;
      case L4:
        return V1_StackUpElevatorPositions.L4;
      default:
        return V1_StackUpElevatorPositions.STOW;
    }
  }

  public Command setVoltage(double volts) {
    return this.runEnd(
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
    return runOnce(() -> this.position = V1_StackUpElevatorPositions.STOW)
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
  public Command runSysID() {
    return Commands.sequence(
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    atGoal(
                        V1_StackUpElevatorPositions.L4.getPosition() - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    atGoal(
                        V1_StackUpElevatorPositions.STOW.getPosition()
                            + Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kForward)
            .until(
                () ->
                    atGoal(
                        V1_StackUpElevatorPositions.L4.getPosition() - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    atGoal(
                        V1_StackUpElevatorPositions.STOW.getPosition()
                            + Units.inchesToMeters(12.0))),
        setPosition(() -> ReefState.STOW));
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
   * Checks if the elevator is at the goal position within a specified tolerance.
   *
   * @param position The target position in meters.
   * @return true if the current position is within the goal tolerance of the target position, false
   *     otherwise.
   */
  private boolean atGoal(double position) {
    return Math.abs(position - inputs.positionMeters)
        <= V1_StackUpElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
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

  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(0.02).andThen(Commands.waitUntil(this::atGoal));
  }

  public BooleanSupplier inFastScoringTolerance() {
    return () -> Math.abs(inputs.positionMeters - inputs.positionGoalMeters) <= 0.03;
  }
}
