package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Manipulator extends SubsystemBase {
  private final ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  public Manipulator(ManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
  }

  public Command runManipulator(double volts) {
    return this.run(
        () -> io.setVoltage(volts));
  }

  public Command intakeCoral() {
    return runManipulator(ManipulatorConstants.VOLTAGES.INTAKE_VOLTS().get());
  }

  public Command scoreCoral() {
    return runManipulator(ManipulatorConstants.VOLTAGES.SCORE_VOLTS().get());
  }

  public boolean hasCoral() {
    return inputs.manipulatorHasCoral;
  }
}
