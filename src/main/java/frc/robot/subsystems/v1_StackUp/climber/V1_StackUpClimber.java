package frc.robot.subsystems.v1_StackUp.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.v1_stackUp.leds.V1_StackUp_LEDs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_StackUpClimber extends SubsystemBase {
  private final V1_StackUpClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private Timer redundantSwitchesTimer;
  private Timer redundantTrustTimer;
  private Timer redundantSwitchTimer;

  @AutoLogOutput(key = "Climber/trustRedundantSwitchOne")
  private boolean trustRedundantSwitchOne;

  @AutoLogOutput(key = "Climber/trustRedundantSwitchTwo")
  private boolean trustRedundantSwitchTwo;

  @AutoLogOutput(key = "Climber/override")
  private boolean override;

  @AutoLogOutput(key = "Climber/SwitchesBroken")
  private boolean broken;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed;

  @AutoLogOutput(key = "Climber/climberReleased")
  private boolean climberDeployed;

  public V1_StackUpClimber(V1_StackUpClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    isClimbed = false;
    redundantSwitchesTimer = new Timer();
    redundantTrustTimer = new Timer();
    redundantSwitchTimer = new Timer();
    trustRedundantSwitchOne = true;
    trustRedundantSwitchTwo = true;
    override = false;
    broken = false;
    climberDeployed = false;
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
    broken = climberSwitchesBroken();
    if (override || broken) {
      return override;
    }

    boolean oldTrustRedundantSwitchOne = trustRedundantSwitchOne;
    boolean oldTrustRedundantSwitchTwo = trustRedundantSwitchTwo;
    if (inputs.redundantSwitchOne != inputs.redundantSwitchTwo) {
      redundantTrustTimer.start();
      trustRedundantSwitchOne = false;
      trustRedundantSwitchTwo = false;
      if (redundantTrustTimer.hasElapsed(
          V1_StackUpClimberConstants.REDUNDANCY_TRUSTING_TIMEOUT_SECONDS)) {
        if (!inputs.redundantSwitchOne) {
          trustRedundantSwitchOne = oldTrustRedundantSwitchOne;
        } else if (!inputs.redundantSwitchTwo) {
          trustRedundantSwitchTwo = oldTrustRedundantSwitchTwo;
        }
      }
    } else {
      trustRedundantSwitchOne = oldTrustRedundantSwitchOne;
      trustRedundantSwitchTwo = oldTrustRedundantSwitchTwo;
      redundantTrustTimer.reset();
    }

    if (inputs.redundantSwitchOne && inputs.redundantSwitchTwo) {
      redundantSwitchesTimer.start();
    } else if (!inputs.redundantSwitchOne && !inputs.redundantSwitchTwo) {
      redundantSwitchesTimer.reset();
    } else if ((inputs.redundantSwitchTwo && trustRedundantSwitchTwo)
        || (inputs.redundantSwitchOne && trustRedundantSwitchOne)) {
      redundantSwitchTimer.start();
    } else if ((!inputs.redundantSwitchTwo && trustRedundantSwitchTwo)
        || (!inputs.redundantSwitchOne && trustRedundantSwitchOne)) {
      redundantSwitchTimer.reset();
    }

    if (trustRedundantSwitchOne && trustRedundantSwitchTwo) {
      return inputs.redundantSwitchOne
          && inputs.redundantSwitchTwo
          && redundantSwitchesTimer.hasElapsed(V1_StackUpClimberConstants.REDUNDANCY_DELAY_SECONDS);
    } else if (trustRedundantSwitchOne) {
      return inputs.redundantSwitchOne
          && redundantSwitchTimer.hasElapsed(V1_StackUpClimberConstants.REDUNDANCY_DELAY_SECONDS);
    } else if (trustRedundantSwitchTwo) {
      return inputs.redundantSwitchTwo
          && redundantSwitchTimer.hasElapsed(V1_StackUpClimberConstants.REDUNDANCY_DELAY_SECONDS);
    } else {
      return false;
    }
  }

  private boolean climberSwitchesBroken() {
    if (broken || (inputs.redundantSwitchOne && inputs.redundantSwitchTwo && !climberDeployed)) {
      V1_StackUp_LEDs.setClimberSensorPanic(true);
      return true;
    } else {
      return false;
    }
  }

  public Command setVoltage(double volts) {
    return this.run(() -> io.setVoltage(volts));
  }

  public Command releaseClimber() {
    return this.runEnd(() -> io.setVoltage(2), () -> io.setVoltage(0))
        .withTimeout(0.1)
        .finallyDo(() -> climberDeployed = true);
  }

  public Command winchClimber() {
    return this.runEnd(() -> io.setVoltage(12), () -> io.setVoltage(0)).until(() -> isClimbed);
  }

  public Command manualDeployOverride(boolean override) { // set using debug board button
    return this.runOnce(() -> this.override = override);
  }
}
