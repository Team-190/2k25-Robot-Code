package frc.robot.subsystems.v1_StackUp.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.v1_stackUp.manipulator.ManipulatorIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_StackUpManipulator extends SubsystemBase {
  private final V1_StackUpManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  private final Timer currentTimer;
  private Rotation2d previousPosition;
  private Rotation2d desiredRotations;

  private boolean assAtSetoint;

  public V1_StackUpManipulator(V1_StackUpManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();

    currentTimer = new Timer();
    previousPosition = inputs.position;
    desiredRotations = new Rotation2d();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput("ASS At Setpoint", assAtSetoint);
    Logger.recordOutput(
        "ASS Setpoint", previousPosition.getRadians() - desiredRotations.getRadians());
  }

  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return Math.abs(inputs.torqueCurrentAmps)
        > V1_StackUpManipulatorConstants.MANIPULATOR_CURRENT_THRESHOLD;
  }

  public Command runManipulator(double volts) {
    return this.runEnd(() -> io.setVoltage(volts), () -> io.setVoltage(0));
  }

  public Command intakeCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> currentTimer.restart()),
        runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.INTAKE_VOLTS().get())
            .until(() -> hasCoral() && currentTimer.hasElapsed(0.25)));
  }

  public Command scoreCoral() {
    return runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.SCORE_VOLTS().get());
  }

  public Command removeAlgae() {
    return runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.REMOVE_ALGAE().get());
  }

  public Command halfScoreCoral() {
    return runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.HALF_VOLTS().get());
  }

  public Command unHalfScoreCoral() {
    return runManipulator(-V1_StackUpManipulatorConstants.VOLTAGES.HALF_VOLTS().get());
  }

  public boolean getManipulatorRotationsOut(Rotation2d rotations) {
    return Math.abs(inputs.position.getRotations())
        >= Math.abs(this.previousPosition.getRotations()) + rotations.getRotations();
  }

  public boolean getManipulatorRotationsIn(Rotation2d rotations) {
    return inputs.position.getRotations()
        <= this.previousPosition.getRotations() - rotations.getRotations();
  }

  public Command toggleAlgaeArm() {
    return Commands.sequence(
        Commands.waitSeconds(0.05),
        Commands.runOnce(
            () -> {
              assAtSetoint = false;
              desiredRotations = V1_StackUpManipulatorConstants.MANIPULATOR_TOGGLE_ARM_ROTATION;
            }),
        Commands.runOnce(() -> this.previousPosition = inputs.position),
        runManipulator(-2)
            .until(
                () ->
                    getManipulatorRotationsIn(
                        V1_StackUpManipulatorConstants.MANIPULATOR_TOGGLE_ARM_ROTATION)),
        Commands.runOnce(() -> assAtSetoint = true));
  }
}
