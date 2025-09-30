package frc.robot.subsystems.v3_Epsilon.superstructure.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakeRollerState;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonIntake {
  private final V3_EpsilonIntakeIO io;
  private final V3_EpsilonIntakeIOInputsAutoLogged inputs;

  @Getter
  @AutoLogOutput(key = "Intake/Pivot Goal")
  private IntakePivotState pivotGoal;

  @Getter
  @AutoLogOutput(key = "Intake/Roller Goal")
  private IntakeRollerState rollerGoal;

  private boolean isClosedLoop;

  public V3_EpsilonIntake(V3_EpsilonIntakeIO io) {
    this.io = io;
    inputs = new V3_EpsilonIntakeIOInputsAutoLogged();

    pivotGoal = IntakePivotState.STOW;
    rollerGoal = IntakeRollerState.STOP;

    isClosedLoop = true;
  }

  /**
   * Periodic function for intake subsystem. Updates inputs from the intake, and then sets the goals
   * for the pivot and roller motors. If the intake is in closed-loop mode, it sets the pivot motor
   * goal to the desired angle. Otherwise, it does not set the pivot motor goal. It always sets the
   * inner and outer roller motor goals to the desired voltage.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (isClosedLoop) {
      io.setPivotGoal(pivotGoal.getAngle());
    }

    io.setInnerRollerVoltage(rollerGoal.getInnerVoltage());
    io.setOuterRollerVoltage(rollerGoal.getOuterVoltage());
  }

  // Double check if this is right

  /**
   * Checks if the intake is currently detecting coral. This is done by checking if both the left
   * and right CAN range sensors are detecting a distance greater than the coral detection
   * threshold.
   *
   * @return True if the intake is detecting coral, false otherwise.
   */
  @AutoLogOutput(key = "Intake/Has Coral")
  public boolean hasCoral() {
    return inputs.leftCANRangeDistanceMeters
            > V3_EpsilonIntakeConstants.INTAKE_CAN_CORAL_DETECTED_THRESHOLD_METERS
        && inputs.rightCANRangeDistanceMeters
            > V3_EpsilonIntakeConstants.INTAKE_CAN_CORAL_DETECTED_THRESHOLD_METERS;
  }

  /**
   * Checks if the pivot motor is at the goal angle. This is done by checking if the absolute
   * difference between the current pivot motor angle and the goal angle is less than the goal
   * tolerance.
   *
   * @return True if the pivot motor is at the goal angle, false otherwise.
   */
  @AutoLogOutput(key = "Intake/At Goal")
  public boolean pivotAtGoal() {
    return Math.abs(inputs.pivotPosition.getRadians() - pivotGoal.getAngle().getRadians())
        < V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.GOAL_TOLERANCE().getRadians();
  }

  /**
   * Checks if the pivot motor is at the goal angle. This is done by checking if the absolute
   * difference between the current pivot motor angle and the goal angle is less than the goal
   * tolerance.
   *
   * @param goal The goal angle to check against.
   * @return True if the pivot motor is at the goal angle, false otherwise.
   */
  public boolean pivotAtGoal(IntakePivotState goal) {
    return Math.abs(inputs.pivotPosition.getRadians() - goal.getAngle().getRadians())
        < V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.GOAL_TOLERANCE().getRadians();
  }

  /**
   * Sets the goal state of the pivot motor. If the intake is currently in open-loop mode, this
   * method will set the isClosedLoop flag to true. It then sets the pivotGoal field to the
   * specified goal state. The pivot motor will then be controlled in closed-loop mode to reach the
   * specified goal state.
   *
   * @param goal The desired IntakePivotState for the pivot motor.
   */
  public void setPivotGoal(IntakePivotState goal) {
    isClosedLoop = true;
    this.pivotGoal = goal;
  }

  /**
   * Sets the voltage for the intake pivot motor.
   *
   * <p>This method is used to set the voltage for the intake pivot motor in open-loop mode. It sets
   * the isClosedLoop flag to false, and then calls the setPivotVoltage method of the IO interface.
   *
   * @param volts The voltage to set for the intake pivot motor.
   */
  public void setPivotVoltage(double volts) {
    isClosedLoop = false;
    io.setPivotVoltage(volts);
  }

  /**
   * Waits until the pivot motor is at the goal angle. This command is created by sequencing a wait
   * command with a waitUntil command. The wait command waits for 0.02 seconds, and the waitUntil
   * command checks if the pivot motor is at the goal angle. The command will block until the pivot
   * motor is at the goal angle.
   *
   * @return A Command that waits until the pivot motor is at the goal angle.
   */
  public Command waitUntilPivotAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::pivotAtGoal));
  }

  public void setRollerGoal(IntakeRollerState rollerGoal) {
    this.rollerGoal = rollerGoal;
    if (hasCoral()
        && Set.of(
                V3_EpsilonIntakeConstants.IntakeRollerState.ALGAE_INTAKE,
                V3_EpsilonIntakeConstants.IntakeRollerState.CORAL_INTAKE,
                V3_EpsilonIntakeConstants.IntakeRollerState.STOP)
            .contains(rollerGoal)) {

      io.setInnerRollerVoltage(0);
      io.setOuterRollerVoltage(0);
    } else {
      io.setInnerRollerVoltage(rollerGoal.getInnerVoltage());
      io.setOuterRollerVoltage(rollerGoal.getOuterVoltage());
    }
  }

  /**
   * Runs the SysId routine for the intake subsystem
   *
   * @param subsystem The subsystem to run the SysId routine on.
   * @return A command that runs the SysId routine.
   */
  private Command sysIdRoutine(Subsystem subsystem) {

    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Intake/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setPivotVoltage(volts.in(Volts)), null, subsystem));

    return Commands.sequence(
        characterizationRoutine
            .quasistatic(Direction.kForward)
            .until(() -> pivotAtGoal(IntakePivotState.HANDOFF)),
        characterizationRoutine
            .quasistatic(Direction.kReverse)
            .until(() -> pivotAtGoal(IntakePivotState.INTAKE_CORAL)),
        characterizationRoutine
            .dynamic(Direction.kForward)
            .until(() -> pivotAtGoal(IntakePivotState.HANDOFF)),
        characterizationRoutine
            .dynamic(Direction.kReverse)
            .until(() -> pivotAtGoal(IntakePivotState.INTAKE_CORAL)));
  }

  /**
   * Gets the current angle of the intake pivot motor.
   *
   * @return The current angle of the intake pivot motor, in radians.
   */
  public Rotation2d getPivotAngle() {
    return inputs.pivotPosition;
  }

  public void updateIntakeGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.updateIntakeGains(kP, kD, kS, kV, kA, kG);
  }

  public void updateIntakeConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateIntakeConstraints(maxAcceleration, cruisingVelocity);
  }
}
