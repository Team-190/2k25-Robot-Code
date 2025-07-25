package frc.robot.subsystems.v1_StackUp.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** The subsystem for the manipulator on the V1 StackUp robot. */
public class V1_StackUpManipulator extends SubsystemBase {
  private final V1_StackUpManipulatorIO io;
  private final V1_StackUpManipulatorIOInputsAutoLogged inputs;

  private final Timer currentTimer;
  private Rotation2d previousPosition;
  private Rotation2d desiredRotations;

  private boolean assAtSetpoint;

  /**
   * Creates a new V1_StackUpManipulator.
   *
   * @param io The hardware interface for the manipulator.
   */
  public V1_StackUpManipulator(V1_StackUpManipulatorIO io) {
    this.io = io;
    inputs = new V1_StackUpManipulatorIOInputsAutoLogged();

    currentTimer = new Timer();
    previousPosition = inputs.position;
    desiredRotations = new Rotation2d();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput("Manipulator/ASS At Setpoint", assAtSetpoint);
    Logger.recordOutput(
        "Manipulator/ASS Setpoint", previousPosition.getRadians() - desiredRotations.getRadians());
  }

  /**
   * Checks if the manipulator is holding a coral.
   *
   * @return True if the manipulator is holding a coral, false otherwise.
   */
  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return Math.abs(inputs.torqueCurrentAmps)
        > V1_StackUpManipulatorConstants.MANIPULATOR_CURRENT_THRESHOLD;
  }

  /**
   * Creates a command to run the manipulator at a specified voltage.
   *
   * @param volts The voltage to run the manipulator at.
   * @return A command to run the manipulator.
   */
  public Command runManipulator(double volts) {
    return this.runEnd(() -> io.setVoltage(volts), () -> io.setVoltage(0));
  }

  /**
   * Creates a command to intake a coral.
   *
   * @return A command to intake a coral.
   */
  public Command intakeCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> currentTimer.restart()),
        runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.INTAKE_VOLTS().get())
            .until(() -> hasCoral() && currentTimer.hasElapsed(0.25)));
  }

  /**
   * Creates a command to score a coral in L4.
   *
   * @return A command to score a coral in L4.
   */
  public Command scoreL4Coral() {
    return runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.L4_VOLTS().get());
  }

  /**
   * Creates a command to score a coral.
   *
   * @return A command to score a coral.
   */
  public Command scoreCoral() {
    return runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.SCORE_VOLTS().get());
  }

  /**
   * Creates a command to score a coral in L1.
   *
   * @return A command to score a coral in L1.
   */
  public Command scoreL1Coral() {
    return runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.L1_VOLTS().get());
  }

  /**
   * Creates a command to remove algae.
   *
   * @return A command to remove algae.
   */
  public Command removeAlgae() {
    return runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.REMOVE_ALGAE().get());
  }

  /**
   * Creates a command to half score a coral.
   *
   * @return A command to half score a coral.
   */
  public Command halfScoreCoral() {
    return runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.HALF_VOLTS().get());
  }

  /**
   * Creates a command to un-half score a coral.
   *
   * @return A command to un-half score a coral.
   */
  public Command unHalfScoreCoral() {
    return runManipulator(-V1_StackUpManipulatorConstants.VOLTAGES.HALF_VOLTS().get());
  }

  /**
   * Checks if the manipulator has rotated out by a certain amount.
   *
   * @param rotations The amount of rotations to check for.
   * @return True if the manipulator has rotated out by the specified amount, false otherwise.
   */
  public boolean getManipulatorRotationsOut(Rotation2d rotations) {
    return Math.abs(inputs.position.getRotations())
        >= Math.abs(this.previousPosition.getRotations()) + rotations.getRotations();
  }

  /**
   * Checks if the manipulator has rotated in by a certain amount.
   *
   * @param rotations The amount of rotations to check for.
   * @return True if the manipulator has rotated in by the specified amount, false otherwise.
   */
  public boolean getManipulatorRotationsIn(Rotation2d rotations) {
    return inputs.position.getRotations()
        <= this.previousPosition.getRotations() - rotations.getRotations();
  }

  /**
   * Creates a command to toggle the algae arm.
   *
   * @return A command to toggle the algae arm.
   */
  public Command toggleAlgaeArm() {
    return Commands.sequence(
        Commands.waitSeconds(0.05),
        Commands.runOnce(
            () -> {
              assAtSetpoint = false;
              desiredRotations = V1_StackUpManipulatorConstants.MANIPULATOR_TOGGLE_ARM_ROTATION;
            }),
        Commands.runOnce(() -> this.previousPosition = inputs.position),
        runManipulator(-2)
            .until(
                () ->
                    getManipulatorRotationsIn(
                        V1_StackUpManipulatorConstants.MANIPULATOR_TOGGLE_ARM_ROTATION)),
        Commands.runOnce(() -> assAtSetpoint = true));
  }
}
