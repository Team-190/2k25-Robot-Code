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
    Logger.processInputs("Manipulators", inputs);
  }

  private Command runManipulator(double volts) {
    return this.run(
        () -> io.setVoltage(volts));
  }

  public Command intakeCoral() {
    return runManipulator(12);
  }

  public Command scoreCoral() {
    return runManipulator(-12);
  }

  public boolean hasCoral() {
    return inputs.manipulatorHasCoral;
  }
}
