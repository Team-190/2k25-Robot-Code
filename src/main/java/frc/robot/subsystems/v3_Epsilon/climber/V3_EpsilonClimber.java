package frc.robot.subsystems.v3_Epsilon.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonClimber extends SubsystemBase {
  private final V3_EpsilonClimberIO io;
  private final V3_EpsilonClimberIOInputsAutoLogged inputs;

  @AutoLogOutput(key = "Climber/override")
  private boolean override;

  @AutoLogOutput(key = "Climber/isClimbed")
  private boolean isClimbed;

  public V3_EpsilonClimber(V3_EpsilonClimberIO io) {
    this.io = io;
    inputs = new V3_EpsilonClimberIOInputsAutoLogged();

    isClimbed = false;

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

    isClimbed = io.isClimbed();

    if (RobotState.isClimberReady()) {
      io.setRollerVoltage(-12);
    }

    ExternalLoggedTracer.record("Climber Total", "Climber/Periodic");
  }

  /**
   * Creates a command to set the voltage of the climber deployment motor.
   *
   * @param volts The voltage to set.
   * @return A command to set the voltage.
   */
  public Command setDeploymentVoltage(double volts) {
    return Commands.run(() -> io.setDeploymentVoltage(volts));
  }

  /**
   * Creates a command to set the voltage of the climber roller motor.
   *
   * @param volts The voltage to set.
   * @return A command to set the voltage.
   */
  public Command setRollerVoltage(double volts) {
    return Commands.run(() -> io.setRollerVoltage(volts));
  }

  /**
   * Creates a command to release the climber. The climber is released by applying voltage until the
   * position is greater than or equal to 20 radians.
   *
   * @return A command to release the climber.
   */
  public Command releaseClimber() {
    return Commands.sequence(
        Commands.runOnce(() -> io.setRollerVoltage(12)),
        this.runEnd(() -> io.setDeploymentVoltage(-1), () -> io.setDeploymentVoltage(0))
            .until(
                () ->
                    inputs.deploymentPosition.getRadians()
                            <= V3_EpsilonClimberConstants.CLIMBER_CLIMBED_DEPLOYED_RADIANS
                        || override)
            .finallyDo(() -> RobotState.setClimberReady(true)));
  }

  /**
   * Creates a command to winch the climber. The climber is winched by applying voltage until the
   * climb is complete.
   *
   * @return A command to winch the climber.
   */
  public Command winchClimber() {
    return Commands.parallel(
        Commands.runOnce(() -> io.setRollerVoltage(0)),
        Commands.runEnd(() -> io.setDeploymentVoltage(-12), () -> io.setDeploymentVoltage(0))
            .until(() -> isClimbed));
  }

  /**
   * Creates a command to manually winch the climber with a lower voltage.
   *
   * @return A command to manually winch the climber.
   */
  public Command winchClimberManual() {
    return Commands.parallel(
        Commands.runOnce(() -> io.setRollerVoltage(0)),
        this.runEnd(() -> io.setDeploymentVoltage(-12.0), () -> io.setDeploymentVoltage(0)));
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
