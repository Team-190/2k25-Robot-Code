package frc.robot.subsystems.v1_gamma.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class V1_GammaManipulator extends SubsystemBase {
  private final V1_GammaManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  public V1_GammaManipulator(V1_GammaManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
  }

  private boolean hasCoral() {
    return inputs.hasCoral;
  }

  public Command runManipulator(double volts) {
    return this.run(() -> io.setVoltage(volts));
  }

  public Command intakeCoral() {
    return runManipulator(V1_GammaManipulatorConstants.VOLTAGES.INTAKE_VOLTS().get())
        .until(() -> hasCoral());
  }

  public Command scoreCoral() {
    return runManipulator(V1_GammaManipulatorConstants.VOLTAGES.SCORE_VOLTS().get())
        .until(() -> !hasCoral());
  }

  private boolean reachedHalfScoreCoral() {
    return inputs.position.getRadians() >= V1_GammaManipulatorConstants.halfScoreThreshold;
  }

  public Command halfScoreCoral() {
    return runManipulator(V1_GammaManipulatorConstants.VOLTAGES.SCORE_VOLTS().get())
      .until(() -> reachedHalfScoreCoral());
  }

}
