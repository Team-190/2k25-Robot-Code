package frc.robot.subsystems.v1_gamma.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_GammaClimber extends SubsystemBase {
  private final V1_GammaClimberIO io;
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

  public V1_GammaClimber(V1_GammaClimberIO io) {
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

    isClimbed = io.isClimbed();
  }

  public boolean climberReady() { 
    if (inputs.redundantSwitchOne != inputs.redundantSwitchOne) {
      redundantTrustTimer.start();
      trustRedundantSwitchOne = false;
      trustRedundantSwitchTwo = false;
      if (redundantTrustTimer.hasElapsed(0.5)) {
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
          && redundantSwitchesTimer.hasElapsed(.25);
    } else if (trustRedundantSwitchOne) {
      return inputs.redundantSwitchOne;
    } else if (trustRedundantSwitchTwo) {
      return inputs.redundantSwitchTwo;
    } else {
      return override;
    }
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

  public Command manualDeployOverride(boolean override) { //set using debug board button
    return this.runOnce(() -> this.override = override);
  }
}
