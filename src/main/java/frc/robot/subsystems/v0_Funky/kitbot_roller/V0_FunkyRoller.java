package frc.robot.subsystems.v0_Funky.kitbot_roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.v0_funky.kitbot_roller.RollerIOInputsAutoLogged;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class V0_FunkyRoller extends SubsystemBase {
  private final V0_FunkyRollerIO io;
  private final RollerIOInputsAutoLogged inputs;

  public V0_FunkyRoller(V0_FunkyRollerIO io) {
    this.io = io;
    inputs = new RollerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
  }

  public Command runRoller(DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.runEnd(
        () -> io.setVoltage(12 * (forward.getAsDouble() - reverse.getAsDouble())),
        () -> io.setVoltage(0),
        this);
  }
}
