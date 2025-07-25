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

  @AutoLogOutput(key = "Climber/trustRedundantSwitchOne")
  private boolean trustRedundantSwitchOne;

  private Timer redundantSwitchesTimer;

  @AutoLogOutput(key = "Climber/trustRedundantSwitchTwo")
  private boolean trustRedundantSwitchTwo;

  private Timer redundantTrustTimer;

  @AutoLogOutput(key = "Climber/override")
  private boolean override;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed;

  /**
   * Creates a new Climber subsystem.
   *
   * @param io The I/O interface for the climber.
   */
  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    isClimbed = false;

    redundantSwitchesTimer = new Timer();
    trustRedundantSwitchOne = true;

    redundantTrustTimer = new Timer();
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

  /**
   * Checks if the climber is ready to be deployed. This is determined by the state of two redundant
   * switches. It includes logic to handle disagreements between the switches and a delay to ensure
   * they are consistently pressed.
   *
   * @return True if the climber is ready, false otherwise.
   */
  public boolean climberReady() {
    if (override) {
      return true;
    }
    if (inputs.redundantSwitchOne != inputs.redundantSwitchOne) {
      redundantTrustTimer.start();
      trustRedundantSwitchOne = false;
      trustRedundantSwitchTwo = false;
      if (redundantTrustTimer.hasElapsed(
          ClimberConstants.CLIMBER_TIMING_CONFIG.REDUNDANCY_TRUSTING_TIMEOUT_SECONDS())) {
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
          && redundantSwitchesTimer.hasElapsed(
              ClimberConstants.CLIMBER_TIMING_CONFIG.REDUNDANCY_DELAY_SECONDS());
    } else if (trustRedundantSwitchOne) {
      return inputs.redundantSwitchOne;
    } else if (trustRedundantSwitchTwo) {
      return inputs.redundantSwitchTwo;
    } else {
      return false;
    }
  }

  /**
   * Creates a command to set the voltage of the climber motor.
   *
   * @param volts The voltage to set.
   * @return A command to set the voltage.
   */
  public Command setVoltage(double volts) {
    return Commands.run(() -> io.setVoltage(volts));
  }

  /**
   * Creates a command to release the climber. The climber is released by applying voltage until the
   * position is greater than or equal to 20 radians.
   *
   * @return A command to release the climber.
   */
  public Command releaseClimber() {
    return this.runEnd(() -> io.setVoltage(1), () -> io.setVoltage(0))
        .until(() -> inputs.positionRadians >= 20);
  }

  /**
   * Creates a command to winch the climber. The climber is winched by applying voltage until the
   * climb is complete.
   *
   * @return A command to winch the climber.
   */
  public Command winchClimber() {
    return Commands.runEnd(() -> io.setVoltage(12), () -> io.setVoltage(0)).until(() -> isClimbed);
  }

  /**
   * Creates a command to manually winch the climber with a lower voltage.
   *
   * @return A command to manually winch the climber.
   */
  public Command winchClimberManual() {
    return this.runEnd(() -> io.setVoltage(4), () -> io.setVoltage(0));
  }

  /**
   * Creates a command to override the climber deployment readiness check.
   *
   * @param override True to override, false otherwise.
   * @return A command to set the override.
   */
  public Command manualDeployOverride(boolean override) { // set using debug board button
    return Commands.runOnce(() -> this.override = override);
  }
}
