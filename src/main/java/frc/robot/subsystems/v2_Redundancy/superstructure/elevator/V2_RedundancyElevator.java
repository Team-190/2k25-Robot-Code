package frc.robot.subsystems.v2_Redundancy.superstructure.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotStateLL;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevatorConstants.V2_RedundancyElevatorPositions;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyElevator {
  private final V2_RedundancyElevatorIO io;
  private final V2_RedundancyElevatorIOInputsAutoLogged inputs;

  @Getter private V2_RedundancyElevatorPositions position;
  private boolean isClosedLoop;

  public V2_RedundancyElevator(V2_RedundancyElevatorIO io) {
    this.io = io;
    inputs = new V2_RedundancyElevatorIOInputsAutoLogged();

    position = V2_RedundancyElevatorPositions.STOW;
    isClosedLoop = true;
  }

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
  public void setPosition() {
    setPosition(() -> RobotStateLL.getOIData().currentReefHeight());
  }

  /**
   * Sets the position of the elevator.
   *
   * @param positionRadians The desired elevator position.
   * @return A command that sets the elevator position.
   */
  public void setPosition(Supplier<ReefState> newPosition) {

    isClosedLoop = true;
    switch (newPosition.get()) {
      case STOW:
        this.position = V2_RedundancyElevatorPositions.STOW;
        break;
      case CORAL_INTAKE:
        this.position = V2_RedundancyElevatorPositions.CORAL_INTAKE;
        break;
      case ALGAE_FLOOR_INTAKE:
        this.position = V2_RedundancyElevatorPositions.ALGAE_INTAKE;
        break;
      case ALGAE_MID:
        this.position = V2_RedundancyElevatorPositions.ALGAE_MID;
        break;
      case ALGAE_INTAKE_TOP:
        this.position = V2_RedundancyElevatorPositions.ALGAE_INTAKE_TOP;
        break;
      case ALGAE_INTAKE_BOTTOM:
        this.position = V2_RedundancyElevatorPositions.ALGAE_INTAKE_BOT;
        break;
      case L1:
        this.position = V2_RedundancyElevatorPositions.L1;
        break;
      case L2:
        this.position = V2_RedundancyElevatorPositions.L2;
        break;
      case L3:
        this.position = V2_RedundancyElevatorPositions.L3;
        break;
      case L4:
        this.position = V2_RedundancyElevatorPositions.L4;
        break;
      case L4_PLUS:
        this.position = V2_RedundancyElevatorPositions.L4_PLUS;
        break;
      case ALGAE_SCORE:
        this.position = V2_RedundancyElevatorPositions.ALGAE_SCORE;
      default:
        break;
    }
    ;
    io.setPositionGoal(this.position.getPosition());
  }

  public V2_RedundancyElevatorPositions getPosition(ReefState newPosition) {
    switch (newPosition) {
      case STOW:
        return V2_RedundancyElevatorPositions.STOW;
      case CORAL_INTAKE:
        return V2_RedundancyElevatorPositions.CORAL_INTAKE;
      case ALGAE_FLOOR_INTAKE:
        return V2_RedundancyElevatorPositions.ALGAE_INTAKE;
      case ALGAE_MID:
        return V2_RedundancyElevatorPositions.ALGAE_MID;
      case ALGAE_INTAKE_TOP:
        return V2_RedundancyElevatorPositions.ALGAE_INTAKE_TOP;
      case ALGAE_INTAKE_BOTTOM:
        return V2_RedundancyElevatorPositions.ALGAE_INTAKE_BOT;
      case L1:
        return V2_RedundancyElevatorPositions.L1;
      case L2:
        return V2_RedundancyElevatorPositions.L2;
      case L3:
        return V2_RedundancyElevatorPositions.L3;
      case L4:
        return V2_RedundancyElevatorPositions.L4;
      case L4_PLUS:
        return V2_RedundancyElevatorPositions.L4_PLUS;
      case ALGAE_SCORE:
        return V2_RedundancyElevatorPositions.ALGAE_SCORE;
      default:
        return V2_RedundancyElevatorPositions.STOW;
    }
  }

  /**
   * Resets the elevator position to the stow position.
   *
   * @return A command that resets the elevator position.
   */
  public Command resetPosition() {
    return Commands.runOnce(() -> this.position = V2_RedundancyElevatorPositions.STOW)
        .andThen(
            Commands.runOnce(
                () ->
                    io.setPosition(
                        V2_RedundancyElevatorConstants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS())));
  }

  /**
   * Runs the SysId routine for the elevator.
   *
   * @return A command to run the SysId routine.
   */
  public Command runSysID(V2_RedundancySuperstructure superstructure) {
    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Elevator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setVoltage(volts.in(Volts)), null, superstructure));

    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(
                () ->
                    atGoal(
                        V2_RedundancyElevatorPositions.L4.getPosition()
                            - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(
                () ->
                    atGoal(
                        V2_RedundancyElevatorPositions.STOW.getPosition()
                            + Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kForward)
            .until(
                () ->
                    atGoal(
                        V2_RedundancyElevatorPositions.L4.getPosition()
                            - Units.inchesToMeters(12.0))),
        characterizationRoutine
            .dynamic(Direction.kReverse)
            .until(
                () ->
                    atGoal(
                        V2_RedundancyElevatorPositions.STOW.getPosition()
                            + Units.inchesToMeters(12.0))),
        Commands.runOnce(() -> setPosition(() -> ReefState.STOW)));
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
        * V2_RedundancyElevatorConstants.ELEVATOR_GEAR_RATIO
        / (2 * Math.PI * V2_RedundancyElevatorConstants.DRUM_RADIUS);
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
        <= V2_RedundancyElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }

  /**
   * Checks if the elevator is at the goal position.
   *
   * @return True if the elevator is at the goal position, false otherwise.
   */
  @AutoLogOutput(key = "Elevator/At Goal")
  public boolean atGoal() {
    return Math.abs(inputs.positionGoalMeters - inputs.positionMeters)
        <= V2_RedundancyElevatorConstants.CONSTRAINTS.goalToleranceMeters().get();
  }

  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(0.02).andThen(Commands.waitUntil(this::atGoal));
  }

  public BooleanSupplier inFastScoringTolerance() {
    return () -> Math.abs(inputs.positionMeters - inputs.positionGoalMeters) <= 0.03;
  }
}
