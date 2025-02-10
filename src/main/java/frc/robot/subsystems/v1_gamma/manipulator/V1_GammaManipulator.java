package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class V1_GammaManipulator extends SubsystemBase {
  private final V1_GammaManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  private final Timer currentTimer;

  public V1_GammaManipulator(V1_GammaManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();

    currentTimer = new Timer();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
  }

  private boolean reachedHalfScoreCoral(Rotation2d currentPosition) {
    return currentPosition.getRadians()
        >= currentPosition.plus(V1_GammaManipulatorConstants.halfScoreRotation).getRadians();
  }

  private boolean reachedStowCoral(Rotation2d currentPosition) {
    return currentPosition.getRadians()
        >= currentPosition.plus(V1_GammaManipulatorConstants.halfScoreRotation).getRadians();
  }

  public boolean hasCoral() {

    return Math.abs(inputs.torqueCurrentAmps)
        > V1_GammaManipulatorConstants.MANIPULATOR_CURRENT_THRESHOLD;
  }

  public Command runManipulator(double volts) {
    return this.runEnd(() -> io.setVoltage(volts), () -> io.setVoltage(0));
  }

  public Command intakeCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> currentTimer.restart()),
        runManipulator(V1_GammaManipulatorConstants.VOLTAGES.INTAKE_VOLTS().get())
            .until(() -> hasCoral() && currentTimer.hasElapsed(0.25)));
  }

  public Command scoreCoral() {
    return runManipulator(V1_GammaManipulatorConstants.VOLTAGES.SCORE_VOLTS().get());
  }

  public Command halfScoreCoral() {
    Rotation2d currentPosition = inputs.position;
    return runManipulator(V1_GammaManipulatorConstants.VOLTAGES.SCORE_VOLTS().get())
        .until(() -> reachedHalfScoreCoral(currentPosition));
  }

  public Command unHalfScoreCoral() {
    Rotation2d currentPosition = inputs.position;
    return runManipulator(V1_GammaManipulatorConstants.VOLTAGES.SCORE_VOLTS().get())
        .until(() -> reachedStowCoral(currentPosition));
  }
}
