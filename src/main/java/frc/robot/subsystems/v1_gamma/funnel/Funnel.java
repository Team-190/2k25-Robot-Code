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
  private final SysIdRoutine serializerCharacterizationRoutine;
  private FunnelState goal;
  private boolean climbing;

  public Funnel(FunnelIO io) {
    this.io = io;
    inputs = new FunnelIOInputsAutoLogged();

    isClosedLoop = true;

    serializerCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Funnel/serializerSysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setSerializerVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);

    if (isClosedLoop) {
      setSerializerPosition(goal.getAngle());
    }

    if (climbing) {
      setSerializerGoal(FunnelState.CLIMB);
    } else {
      if (hasCoral()) {
        setSerializerGoal(FunnelState.CLOSED);
      } else {
        setSerializerGoal(FunnelState.OPENED);
      }
    }
  }

  public Command setClimbing(boolean climbing) {
    return runOnce(() -> this.climbing = climbing);
  }

  public Command setSerializerGoal(FunnelState goal) {
    return runOnce(
        () -> {
          isClosedLoop = true;
          this.goal = goal;
        });
  }

  public Command setRollerVoltage(double volts) {
    return run(() -> io.setRollerVoltage(volts));
  }

  public Command setSerializerVoltage(double volts) {
    isClosedLoop = false;
    return run(() -> io.setSerializerVoltage(volts));
  }

  public Command setSerializerPosition(double radians) {
    isClosedLoop = true;
    return run(() -> io.setSerializerPosition(radians));
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
        serializerCharacterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(4),
        serializerCharacterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(4), 
        serializerCharacterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(4), 
        serializerCharacterizationRoutine.dynamic(Direction.kReverse));
  }

  @AutoLogOutput(key = "Funnel/Serializer Motor At Goal")
  public boolean serializerMotorAtGoal() {
    return io.atSerializerGoal();
  }
}
