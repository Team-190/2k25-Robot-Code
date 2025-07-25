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
import frc.robot.RobotStateLL;
import frc.robot.RobotStateLL.RobotMode;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelRollerState;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Funnel {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs;

  private double debounceTimestamp;

  @AutoLogOutput(key = "Funnel/ClapDaddy Goal")
  private FunnelState clapDaddyGoal;

  private boolean isClosedLoop;

  public Funnel(FunnelIO io) {
    this.io = io;
    inputs = new FunnelIOInputsAutoLogged();

    debounceTimestamp = Timer.getFPGATimestamp();
    clapDaddyGoal = FunnelState.OPENED;
    isClosedLoop = true;
  }

  private void periodic() {
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
   * Sets the goal state of the clapDaddy.
   *
   * @param goal The desired FunnelState.
   * @return A command to set the clapDaddy goal.
   */
  private void setClapDaddyGoal(FunnelState goal) {
    isClosedLoop = true;
    clapDaddyGoal = goal;
  }

  /**
   * Runs the SysId routine for the clapDaddy.
   *
   * @return A command to run the SysId routine.
   */
  private Command sysIdRoutine(Subsystem subsystem) {
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
  private boolean hasCoral() {
    return inputs.hasCoral && Timer.getFPGATimestamp() > debounceTimestamp + 0.05;
  }

  /**
   * Checks if the clapDaddy motor is at the goal position.
   *
   * @return True if the clapDaddy motor is at the goal, false otherwise.
   */
  @AutoLogOutput(key = "Funnel/At Goal")
  private boolean atGoal() {
    return io.atClapDaddyPositionGoal();
  }

  /**
   * Gets the current angle of the clapDaddy.
   *
   * @return The current angle as a Rotation2d.
   */
  private Rotation2d getAngle() {
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
  private void updateGains(double kP, double kD, double kS, double kV, double kA) {
    io.updateGains(kP, kD, kS, kV, kA);
  }

  /**
   * Updates the motion constraints for the clapDaddy.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  private void updateConstraints(double maxAcceleration, double maxVelocity) {
    io.updateConstraints(maxAcceleration, maxVelocity);
  }

  public class FunnelFSM {
    @Setter private boolean manipulatorHasCoral;

    @Getter
    @AutoLogOutput(key = "Funnel/Roller Goal")
    private FunnelRollerState rollerGoal;

    public FunnelFSM() {
      manipulatorHasCoral = false;
      rollerGoal = FunnelRollerState.STOP;
    }

    public FunnelState getClapDaddyGoal() {
      return clapDaddyGoal;
    }

    public void periodic() {
      ExternalLoggedTracer.reset();
      Funnel.this.periodic();
      io.setRollerVoltage(rollerGoal.getVoltage());
      if (RobotStateLL.isIntakingCoral()) {
        if (hasCoral()) {
          setClapDaddyGoal(FunnelState.CLOSED);
        }
        if (manipulatorHasCoral) {
          setClapDaddyGoal(FunnelState.OPENED);
        }
      }

      if (RobotMode.auto() && RobotStateLL.isAutoClapOverride()) {
        setClapDaddyGoal(FunnelState.CLOSED);
      }
      ExternalLoggedTracer.record("Funnel Periodic", "Funnel/Periodic");
    }

    public void setClapDaddyGoal(FunnelState goal) {
      Funnel.this.setClapDaddyGoal(goal);
    }

    public void setRollerGoal(FunnelRollerState state) {
      rollerGoal = state;
    }

    public Command sysIdRoutine(V2_RedundancySuperstructure superstructure) {
      return Commands.sequence(
          superstructure.runGoal(V2_RedundancySuperstructureStates.OVERRIDE),
          Funnel.this.sysIdRoutine(superstructure));
    }

    public boolean hasCoral() {
      return Funnel.this.hasCoral();
    }

    public boolean atGoal() {
      return Funnel.this.atGoal();
    }

    public Rotation2d getAngle() {
      return Funnel.this.getAngle();
    }

    public void updateGains(double kP, double kD, double kS, double kV, double kA) {
      Funnel.this.updateGains(kP, kD, kS, kV, kA);
    }

    public void updateConstraints(double maxAcceleration, double maxVelocity) {
      Funnel.this.updateConstraints(maxAcceleration, maxVelocity);
    }
  }

  public class FunnelCSB extends SubsystemBase {

    @Override
    public void periodic() {
      ExternalLoggedTracer.reset();
      Funnel.this.periodic();
      ExternalLoggedTracer.record("Funnel Periodic", "Funnel/Periodic");
    }

    public Command setClapDaddyGoal(FunnelState goal) {
      return Commands.runOnce(() -> Funnel.this.setClapDaddyGoal(goal));
    }

    public FunnelState getClapDaddyGoal() {
      return clapDaddyGoal;
    }

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

    public Command sysIdRoutine() {
      return Funnel.this.sysIdRoutine(this);
    }

    public boolean hasCoral() {
      return Funnel.this.hasCoral();
    }

    public boolean atGoal() {
      return Funnel.this.atGoal();
    }

    public Rotation2d getAngle() {
      return Funnel.this.getAngle();
    }

    public void updateGains(double kP, double kD, double kS, double kV, double kA) {
      Funnel.this.updateGains(kP, kD, kS, kV, kA);
    }

    public void updateConstraints(double maxAcceleration, double maxVelocity) {
      Funnel.this.updateConstraints(maxAcceleration, maxVelocity);
    }
  }

  public FunnelFSM getFSM() {
    return new FunnelFSM();
  }

  public FunnelCSB getCSB() {
    return new FunnelCSB();
  }
}
