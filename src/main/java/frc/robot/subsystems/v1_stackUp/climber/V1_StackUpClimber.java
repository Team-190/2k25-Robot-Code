package frc.robot.subsystems.v1_stackUp.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_StackUpClimber extends SubsystemBase {
  private final V1_StackUpClimberIO io;
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

  public V1_StackUpClimber(V1_StackUpClimberIO io) {
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
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    Logger.recordOutput("Climber/redundantSwitchesTimer", redundantSwitchesTimer.get());
    Logger.recordOutput("Climber/redundantTrustTimer", redundantTrustTimer.get());
    Logger.recordOutput("Climber/Ready", climberReady());

    isClimbed = io.isClimbed();
  }

  public boolean climberReady() {
    if (override) {
      return true;
    }
    if (inputs.redundantSwitchOne != inputs.redundantSwitchOne) {
      redundantTrustTimer.start();
      trustRedundantSwitchOne = false;
      trustRedundantSwitchTwo = false;
      if (redundantTrustTimer.hasElapsed(
          V1_StackUpClimberConstants.REDUNDANCY_TRUSTING_TIMEOUT_SECONDS)) {
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
          && redundantSwitchesTimer.hasElapsed(V1_StackUpClimberConstants.REDUNDANCY_DELAY_SECONDS);
    } else if (trustRedundantSwitchOne) {
      return inputs.redundantSwitchOne;
    } else if (trustRedundantSwitchTwo) {
      return inputs.redundantSwitchTwo;
    } else {
      return false;
    }
  }

  public Command setVoltage(double volts) {
    return this.run(() -> io.setVoltage(volts));
  }

  public Command releaseClimber() {
    return this.runEnd(() -> io.setVoltage(2), () -> io.setVoltage(0)).withTimeout(0.1);
  }

  public Command winchClimber() {
    return this.runEnd(() -> io.setVoltage(12), () -> io.setVoltage(0)).until(() -> isClimbed);
  }

  public Command manualDeployOverride(boolean override) { // set using debug board button
    return this.runOnce(() -> this.override = override);
  }

  public Command incrementWinchClimber() {
    return this.runEnd(() -> io.setVoltage(12), () -> io.setVoltage(0)).withTimeout(0.04);
  }

  public Command decrementWinchClimber() {
    return this.runEnd(() -> io.setVoltage(-12), () -> io.setVoltage(0)).withTimeout(0.04);
  }
}
