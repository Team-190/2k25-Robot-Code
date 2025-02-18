package frc.robot.subsystems.v1_gamma.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_GammaClimber extends SubsystemBase {
  private final V1_GammaClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private Timer reduntantSwitchTimer;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed;

  public V1_GammaClimber(V1_GammaClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    isClimbed = false;
    reduntantSwitchTimer = new Timer();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    isClimbed = io.isClimbed();
  }

  public boolean climberReady() {
    if (inputs.redundantSwitchOne && inputs.redundantSwitchOne) {
      reduntantSwitchTimer.start();
    } else {
      reduntantSwitchTimer.reset();
    }
    return inputs.redundantSwitchOne
        && inputs.redundantSwitchOne
        && reduntantSwitchTimer.hasElapsed(.25);
  }

  public Command setVoltage(double volts) {
    return this.run(() -> io.setVoltage(volts));
  }

  public Command releaseClimber() {
    return this.runEnd(() -> io.setVoltage(4), () -> io.setVoltage(0)).withTimeout(0.1);
  }

  public Command winchClimber() {
    return this.runEnd(() -> io.setVoltage(12), () -> io.setVoltage(0)).until(() -> isClimbed);
  }

  public Command incrementWinchClimber() {
    return this.runEnd(() -> io.setVoltage(12), () -> io.setVoltage(0)).withTimeout(0.04);
  }

  public Command decrementWinchClimber() {
    return this.runEnd(() -> io.setVoltage(-12), () -> io.setVoltage(0)).withTimeout(0.04);
  }
}
