package frc.robot.subsystems.v3_Epsilon.manipulator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonManipulator extends SubsystemBase {
  private final V3_EpsilonManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  private final Timer currentTimer;
  private Rotation2d previousPosition;
  private Rotation2d desiredRotations;

  public V3_EpsilonManipulator(V3_EpsilonManipulatorIO io) {
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
  }

  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return Math.abs(inputs.torqueCurrentAmps)
        > V3_EpsilonManipulatorConstants.MANIPULATOR_CURRENT_THRESHOLD;
  }

  public Command runManipulator(double volts) {
    return this.runEnd(() -> io.setVoltage(volts), () -> io.setVoltage(0));
  }
}
