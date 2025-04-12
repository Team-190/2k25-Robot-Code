package frc.robot.subsystems.shared.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private Timer redundantSwitchesTimer;
  private Timer redundantTrustTimer;

  @AutoLogOutput(key = "Climber/trustRedundantSwitchOne")
  private boolean trustRedundantSwitchOne;

  @AutoLogOutput(key = "Climber/trustRedundantSwitchTwo")
  private boolean trustRedundantSwitchTwo;

  @AutoLogOutput(key = "Climber/override")
  private boolean override;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    isClimbed = false;
    redundantSwitchesTimer = new Timer();
    redundantTrustTimer = new Timer();
    trustRedundantSwitchOne = true;
    trustRedundantSwitchTwo = true;
    override = false;
  }

  @Override
  public void periodic() {
    ExternalLoggedTracer.reset();
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Climber Input Update", "Climber/Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Climber", inputs);
    InternalLoggedTracer.record("Climber Input Processing", "Climber/Periodic");

    InternalLoggedTracer.reset();
    Logger.recordOutput("Climber/redundantSwitchesTimer", redundantSwitchesTimer.get());
    Logger.recordOutput("Climber/redundantTrustTimer", redundantTrustTimer.get());
    Logger.recordOutput("Climber/Ready", climberReady());
    InternalLoggedTracer.record("Logging", "Climber/Periodic");

    isClimbed = io.isClimbed();
    ExternalLoggedTracer.record("Climber Total", "Climber/Periodic");
  }

  public boolean climberReady() {
    if (override) {
      return true;
    }
    if (inputs.redundantSwitchOne != inputs.redundantSwitchOne) {
      redundantTrustTimer.start();
      trustRedundantSwitchOne = false;
      trustRedundantSwitchTwo = false;
      if (redundantTrustTimer.hasElapsed(ClimberConstants.REDUNDANCY_TRUSTING_TIMEOUT_SECONDS)) {
        if (inputs.redundantSwitchOne) {
          trustRedundantSwitchOne = true;
        } else if (inputs.redundantSwitchOne) {
          trustRedundantSwitchTwo = true;
        }
      }
    } else {
      trustRedundantSwitchOne = true;
      trustRedundantSwitchTwo = true;
      redundantTrustTimer.reset();
    }

    if (inputs.redundantSwitchOne && inputs.redundantSwitchOne) {
      redundantSwitchesTimer.start();
    } else {
      redundantSwitchesTimer.reset();
    }

    if (trustRedundantSwitchOne && trustRedundantSwitchTwo) {
      return inputs.redundantSwitchOne
          && inputs.redundantSwitchTwo
          && redundantSwitchesTimer.hasElapsed(ClimberConstants.REDUNDANCY_DELAY_SECONDS);
    } else if (trustRedundantSwitchOne) {
      return inputs.redundantSwitchOne;
    } else if (trustRedundantSwitchTwo) {
      return inputs.redundantSwitchTwo;
    } else {
      return false;
    }
  }

  public Command setVoltage(double volts) {
    return Commands.run(() -> io.setVoltage(volts));
  }

  public Command releaseClimber() {
    return this.runEnd(() -> io.setVoltage(1), () -> io.setVoltage(0))
        .until(() -> inputs.positionRadians >= 20);
  }

  public Command winchClimber() {
    return Commands.runEnd(() -> io.setVoltage(12), () -> io.setVoltage(0)).until(() -> isClimbed);
  }

  public Command winchClimberManual() {
    return this.runEnd(() -> io.setVoltage(4), () -> io.setVoltage(0));
  }

  public Command manualDeployOverride(boolean override) { // set using debug board button
    return Commands.runOnce(() -> this.override = override);
  }
}
