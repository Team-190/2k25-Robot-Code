package frc.robot.subsystems.shared.funnel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.util.InternalLoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public abstract class Funnel extends SubsystemBase {
  protected FunnelIO io;
  protected FunnelIOInputsAutoLogged inputs;

  protected boolean isClosedLoop;

  @Getter
  @AutoLogOutput(key = "Funnel/ClapDaddy Goal")
  protected FunnelState clapDaddyGoal;

  protected double debounceTimestamp;

  public Funnel(FunnelIO io, FunnelIOInputsAutoLogged inputs) {
    this.io = io;
    this.inputs = inputs;

    isClosedLoop = true;
    clapDaddyGoal = FunnelState.OPENED;

    debounceTimestamp = Timer.getFPGATimestamp();
  }

  public void periodic() {
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Funnel/Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Funnel", inputs);
    InternalLoggedTracer.record("Process Inputs", "Funnel/Periodic");

    InternalLoggedTracer.reset();
    if (isClosedLoop) {
      io.setClapDaddyGoal(clapDaddyGoal.getAngle());
    }
    InternalLoggedTracer.record("Set Funnel Goal", "Funnel/Periodic");

    InternalLoggedTracer.reset();
    if (!inputs.hasCoral) {
      debounceTimestamp = Timer.getFPGATimestamp();
    }
    InternalLoggedTracer.record("Update debounce Timestamp", "Funnel/Periodic");
  }

  /**
   * Runs the SysId routine for the clapDaddy.
   *
   * @return A command to run the SysId routine.
   */
  public Command sysIdRoutine(Subsystem subsystem) {
    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(1),
                (state) -> Logger.recordOutput("Funnel/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setClapDaddyVoltage(volts.in(Volts)), null, subsystem));
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(0.25),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(0.25),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(0.25),
        characterizationRoutine.dynamic(Direction.kReverse));
  }

  /**
   * Checks if the funnel has coral.
   *
   * @return True if the funnel has coral, false otherwise.
   */
  public boolean hasCoral() {
    return inputs.hasCoral && Timer.getFPGATimestamp() > debounceTimestamp + 0.05;
  }

  /**
   * Checks if the clapDaddy motor is at the goal position.
   *
   * @return True if the clapDaddy motor is at the goal, false otherwise.
   */
  @AutoLogOutput(key = "Funnel/At Goal")
  public boolean atGoal() {
    return io.atClapDaddyPositionGoal();
  }

  public Rotation2d getAngle() {
    return inputs.clapDaddyAbsolutePosition;
  }

  /**
   * Updates the PID gains for the clapDaddy.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   */
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    io.updateGains(kP, kD, kS, kV, kA);
  }

  /**
   * Updates the motion constraints for the clapDaddy.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    io.updateConstraints(maxAcceleration, maxVelocity);
  }
}
