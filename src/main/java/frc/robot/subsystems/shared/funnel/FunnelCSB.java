package frc.robot.subsystems.shared.funnel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.util.ExternalLoggedTracer;
import java.util.function.BooleanSupplier;

public class FunnelCSB extends Funnel {

  public FunnelCSB(FunnelIO io) {
    super(io, new FunnelIOInputsAutoLogged());

    debounceTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    ExternalLoggedTracer.reset();

    super.periodic();
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
          super.isClosedLoop = true;
          clapDaddyGoal = goal;
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
              clapDaddyGoal = FunnelState.OPENED;
              io.setRollerVoltage(0.0);
            });
  }

  public Command funnelClosedOverride() {
    return this.runEnd(
        () -> {
          clapDaddyGoal = FunnelState.CLOSED;
          io.setRollerVoltage(12);
        },
        () -> {
          clapDaddyGoal = FunnelState.OPENED;
          io.setRollerVoltage(0);
        });
  }

  /**
   * Runs the SysId routine for the clapDaddy.
   *
   * @return A command to run the SysId routine.
   */
  public Command sysIdRoutine() {
    return super.sysIdRoutine(this);
  }

  public Command setFunnelVoltage(double volts) {
    return Commands.run(() -> io.setClapDaddyVoltage(volts));
  }
}
