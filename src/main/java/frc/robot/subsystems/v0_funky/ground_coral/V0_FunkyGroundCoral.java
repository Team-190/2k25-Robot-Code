package frc.robot.subsystems.v0_funky.ground_coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class V0_FunkyGroundCoral extends SubsystemBase {
  private final V0_FunkyGroundCoralIO io;
  private final GroundCoralIOInputsAutoLogged inputs;

  public V0_FunkyGroundCoral(V0_FunkyGroundCoralIO io) {
    this.io = io;
    inputs = new GoundCoralIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GroundCoral", inputs);
  }

  public Command runRoller(DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.runEnd(
        () -> io.setVoltage(12 * (forward.getAsDouble() - reverse.getAsDouble())),
        () -> io.setVoltage(0),
        this);
  }
}
