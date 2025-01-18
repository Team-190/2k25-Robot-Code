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
  private final SysIdRoutine intakeCharacterizationRoutine;
  private final SysIdRoutine crabCharacterizationRoutine;
  private FunnelState goal;
  private boolean climbing;

  public Funnel(FunnelIO io) {
    this.io = io;
    inputs = new FunnelIOInputsAutoLogged();

    isClosedLoop = true;
    intakeCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Funnel/intakeSysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setIntakeVoltage(volts.in(Volts)), null, this));

    crabCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Funnel/crabSysIDState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setCrabVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);

    if (isClosedLoop) {
      setCrabPosition(goal.getAngle());
    }

    if (climbing) {
      setCrabGoal(FunnelState.CLIMB);
    } else {
      if (hasCoral()) {
        setCrabGoal(FunnelState.CLOSED);
      } else {
        setCrabGoal(FunnelState.OPENED);
      }
    }
  }

  public Command setClimbing(boolean climbing) {
    return runOnce(() -> this.climbing = climbing);
  }

  public Command setCrabGoal(FunnelState goal) {
    return runOnce(
        () -> {
          isClosedLoop = true;
          this.goal = goal;
        });
  }

  public Command setIntakeVoltage(double volts) {
    return run(() -> io.setIntakeVoltage(volts));
  }

  public Command setCrabVoltage(double volts) {
    isClosedLoop = false;
    return run(() -> io.setCrabVoltage(volts));
  }

  public Command setCrabPosition(double radians) {
    isClosedLoop = true;
    return run(() -> io.setCrabPosition(radians));
  }

  public Command setIntakeVelocity(double radiansPerSecond) {
    return run(() -> io.setIntakeVelocity(radiansPerSecond));
  }

  public Command stopIntake() {
    return runOnce(io::stopIntake);
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  public Command sysIdRoutine() {
    return Commands.sequence(
        runOnce(() -> isClosedLoop = false),
        Commands.parallel(
            crabCharacterizationRoutine.quasistatic(Direction.kForward),
            intakeCharacterizationRoutine.quasistatic(Direction.kForward)),
        Commands.waitSeconds(4),
        Commands.parallel(
            crabCharacterizationRoutine.quasistatic(Direction.kReverse),
            intakeCharacterizationRoutine.quasistatic(Direction.kReverse)),
        Commands.waitSeconds(4),
        Commands.parallel(
            crabCharacterizationRoutine.dynamic(Direction.kForward),
            intakeCharacterizationRoutine.dynamic(Direction.kForward)),
        Commands.waitSeconds(4),
        Commands.parallel(
            crabCharacterizationRoutine.dynamic(Direction.kReverse),
            intakeCharacterizationRoutine.dynamic(Direction.kReverse)));
  }

  @AutoLogOutput(key = "Funnel/Crab Motor At Goal")
  public boolean crabMotorAtGoal() {
    return io.atCrabGoal();
  }

  @AutoLogOutput(key = "Funnel/Intake Motor At Goal")
  public boolean intakeMotorAtGoal() {
    return io.atIntakeGoal();
  }
}
