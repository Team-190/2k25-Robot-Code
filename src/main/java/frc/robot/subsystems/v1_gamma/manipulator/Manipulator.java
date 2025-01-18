package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
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

  public Command runManipulator(DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(
        () -> io.setVoltage(12 * (forward.getAsDouble() - reverse.getAsDouble())), this);
  }

  public boolean hasLeft() {
    return inputs.coralHasLeft;
  }
}
