package frc.robot.subsystems.v1_gamma.funnel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v1_gamma.funnel.FunnelConstants.FunnelState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs;
  private boolean isClosedLoop;
  private final SysIdRoutine rollerCharacterizationRoutine;
  private final SysIdRoutine clapperCharacterizationRoutine;
  private FunnelState goal;
  private boolean climbing;

  public Funnel(FunnelIO io) {
    this.io = io;
    inputs = new FunnelIOInputsAutoLogged();

    isClosedLoop = true;
    rollerCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Funnel/rollerSysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setRollerVoltage(volts.in(Volts)), null, this));

    clapperCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Funnel/clapperSysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setClapperVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);

    if (isClosedLoop) {
      setClapperPosition(goal.getAngle());
    }

    if (climbing) {
      setClapperGoal(FunnelState.CLIMB);
    } else {
      if (hasCoral()) {
        setClapperGoal(FunnelState.CLOSED);
      } else {
        setClapperGoal(FunnelState.OPENED);
      }
    }
  }

  public Command setClimbing(boolean climbing) {
    return runOnce(() -> this.climbing = climbing);
  }

  public Command setClapperGoal(FunnelState goal) {
    return runOnce(
        () -> {
          isClosedLoop = true;
          this.goal = goal;
        });
  }

  public Command setRollerVoltage(double volts) {
    return run(() -> io.setRollerVoltage(volts));
  }

  public Command setClapperVoltage(double volts) {
    isClosedLoop = false;
    return run(() -> io.setClapperVoltage(volts));
  }

  public Command setClapperPosition(double radians) {
    isClosedLoop = true;
    return run(() -> io.setClapperPosition(radians));
  }

  public Command setRollerVelocity(double radiansPerSecond) {
    return run(() -> io.setRollerVelocity(radiansPerSecond));
  }

  public Command stopRoller() {
    return runOnce(io::stopRoller);
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  public Command sysIdRoutine() {
    return Commands.sequence(
        runOnce(() -> isClosedLoop = false),
        Commands.parallel(
            clapperCharacterizationRoutine.quasistatic(Direction.kForward),
            rollerCharacterizationRoutine.quasistatic(Direction.kForward)),
        Commands.waitSeconds(4),
        Commands.parallel(
            clapperCharacterizationRoutine.quasistatic(Direction.kReverse),
            rollerCharacterizationRoutine.quasistatic(Direction.kReverse)),
        Commands.waitSeconds(4),
        Commands.parallel(
            clapperCharacterizationRoutine.dynamic(Direction.kForward),
            rollerCharacterizationRoutine.dynamic(Direction.kForward)),
        Commands.waitSeconds(4),
        Commands.parallel(
            clapperCharacterizationRoutine.dynamic(Direction.kReverse),
            rollerCharacterizationRoutine.dynamic(Direction.kReverse)));
  }

  @AutoLogOutput(key = "Funnel/Clapper Motor At Goal")
  public boolean clapperMotorAtGoal() {
    return io.atClapperGoal();
  }

  @AutoLogOutput(key = "Funnel/Roller Motor At Goal")
  public boolean rollerMotorAtGoal() {
    return io.atRollerGoal();
  }
}
