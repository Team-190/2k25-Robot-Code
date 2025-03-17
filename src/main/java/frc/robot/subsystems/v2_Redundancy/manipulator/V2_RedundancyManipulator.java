package frc.robot.subsystems.v2_Redundancy.manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyManipulator extends SubsystemBase {
  private final V2_RedundancyManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  private final Timer currentTimer;

  public V2_RedundancyManipulator(V2_RedundancyManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();

    currentTimer = new Timer();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
  }

  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return Math.abs(inputs.torqueCurrentAmps)
        > V2_RedundancyManipulatorConstants.MANIPULATOR_CURRENT_THRESHOLD;
  }

  public Command runManipulator(double volts) {
    return this.runEnd(() -> io.setVoltage(volts), () -> io.setVoltage(0));
  }

  public Command intakeCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> currentTimer.restart()),
        runManipulator(V2_RedundancyManipulatorConstants.VOLTAGES.INTAKE_VOLTS().get())
            .until(() -> hasCoral() && currentTimer.hasElapsed(0.25)));
  }

  public Command scoreCoral() {
    return runManipulator(V2_RedundancyManipulatorConstants.VOLTAGES.SCORE_VOLTS().get());
  }

  public Command scoreL1Coral() {
    return runManipulator(V2_RedundancyManipulatorConstants.VOLTAGES.L1_VOLTS().get());
  }

  public Command halfScoreCoral() {
    return runManipulator(V2_RedundancyManipulatorConstants.VOLTAGES.HALF_VOLTS().get());
  }

  public Command unHalfScoreCoral() {
    return runManipulator(-V2_RedundancyManipulatorConstants.VOLTAGES.HALF_VOLTS().get());
  }
}
