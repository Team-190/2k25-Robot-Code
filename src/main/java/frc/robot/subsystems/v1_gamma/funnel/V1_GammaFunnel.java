package frc.robot.subsystems.v1_gamma.funnel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelConstants.FunnelState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V1_GammaFunnel extends SubsystemBase {
  private final V1_GammaFunnelIO io;
  private final FunnelIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;
  private FunnelState goal;

  private boolean isClimbing;
  private boolean isClosedLoop;

  public V1_GammaFunnel(V1_GammaFunnelIO io) {
    this.io = io;
    inputs = new FunnelIOInputsAutoLogged();

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(3),
                (state) -> Logger.recordOutput("Funnel/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setClapDaddyVoltage(volts.in(Volts)), null, this));
    goal = FunnelState.OPENED;

    isClimbing = false;
    isClosedLoop = true;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);

    if (isClosedLoop) {
      io.setClapDaddyPosition(goal.getAngle());
    }

    if (isClimbing) {
      setClapDaddyGoal(FunnelState.CLIMB);
    }
  }

  /**
   * Sets the goal state of the clapDaddy.
   *
   * @param goal The desired FunnelState.
   * @return A command to set the clapDaddy goal.
   */
  private Command setClapDaddyGoal(FunnelState goal) {
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
        setRollerVoltage(12.0));
  }

  /**
   * Stops the roller.
   *
   * @return A command to stop the roller.
   */
  public Command stopRoller() {
    return Commands.runOnce(io::stopRoller);
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
   * Sets the climbing state of the funnel.
   *
   * @param climbing True if the funnel is climbing, false otherwise.
   * @return A command to set the climbing state.
   */
  public Command setClimbing(boolean climbing) {
    return Commands.runOnce(() -> this.isClimbing = climbing);
  }

  /**
   * Checks if the funnel has coral.
   *
   * @return True if the funnel has coral, false otherwise.
   */
  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  /**
   * Checks if the clapDaddy motor is at the goal position.
   *
   * @return True if the clapDaddy motor is at the goal, false otherwise.
   */
  @AutoLogOutput(key = "Funnel/At Goal")
  public boolean atGoal() {
    return io.atClapDaddyGoal();
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
