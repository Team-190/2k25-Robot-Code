package frc.robot.subsystems.shared.funnel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;
  private double debounceTimestamp;
  private FunnelState goal;

  private boolean isClosedLoop;

  public Funnel(FunnelIO io) {
    this.io = io;
    inputs = new FunnelIOInputsAutoLogged();

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(1),
                (state) -> Logger.recordOutput("Funnel/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setClapDaddyVoltage(volts.in(Volts)), null, this));
    debounceTimestamp = Timer.getFPGATimestamp();
    goal = FunnelState.OPENED;

    isClosedLoop = true;
  }

  @Override
  public void periodic() {
    ExternalLoggedTracer.reset();
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Funnel/Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Funnel", inputs);
    InternalLoggedTracer.record("Process Inputs", "Funnel/Periodic");

    InternalLoggedTracer.reset();
    if (isClosedLoop) {
      io.setClapDaddyGoal(goal.getAngle());
    }
    InternalLoggedTracer.record("Set Funnel Goal", "Funnel/Periodic");

    InternalLoggedTracer.reset();
    if (!inputs.hasCoral) {
      debounceTimestamp = Timer.getFPGATimestamp();
    }
    InternalLoggedTracer.record("Update debounce Timestamp", "Funnel/Periodic");
    ExternalLoggedTracer.record("Funnel Periodic", "Funnel/Periodic");
  }

  /**
   * Sets the goal state of the clapDaddy.
   *
   * @param goal The desired FunnelState.
   * @return A command to set the clapDaddy goal.
   */
  public Command setClapDaddyGoal(FunnelState goal) {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          this.goal = goal;
        });
  }

  /**
   * Sets the voltage of the roller.
   *
   * @param volts The desired voltage.
   * @return A command to set the roller voltage.
   */
  private Command setRollerVoltage(double volts) {
    return Commands.run(() -> io.setRollerVoltage(volts));
  }

  public Command intakeCoral(BooleanSupplier coralLocked) {
    return Commands.race(
            Commands.sequence(
                setClapDaddyGoal(FunnelState.OPENED),
                Commands.waitUntil(() -> hasCoral()),
                setClapDaddyGoal(FunnelState.CLOSED),
                Commands.waitUntil(coralLocked)),
            setRollerVoltage(12.0))
        .finallyDo(
            () -> {
              goal = FunnelState.OPENED;
              io.setRollerVoltage(0.0);
            });
  }

  /**
   * Stops the roller.
   *
   * @return A command to stop the roller.
   */
  public Command stopRoller() {
    return Commands.runOnce(io::stopRoller);
  }

  public Command funnelClosedOverride() {
    return Commands.runEnd(
        () -> {
          goal = FunnelState.CLOSED;
          io.setRollerVoltage(12);
        },
        () -> {
          goal = FunnelState.OPENED;
          io.setRollerVoltage(0);
        });
  }

  /**
   * Runs the SysId routine for the clapDaddy.
   *
   * @return A command to run the SysId routine.
   */
  public Command sysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(4),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(4),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(4),
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

  public Command setFunnelVoltage(double volts) {
    return Commands.run(() -> io.setClapDaddyVoltage(volts));
  }
}
