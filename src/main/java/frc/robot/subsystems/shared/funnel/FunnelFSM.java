package frc.robot.subsystems.shared.funnel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FunnelFSM {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs;

  private double debounceTimestamp;
  @Setter private boolean manipulatorHasCoral;

  @Getter
  @AutoLogOutput(key = "Funnel/ClapDaddy Goal")
  private FunnelState clapDaddyGoal;

  @Getter
  @AutoLogOutput(key = "Funnel/Roller Goal")
  private FunnelRollerState rollerGoal;

  private boolean isClosedLoop;

  public FunnelFSM(FunnelIO io) {
    this.io = io;
    inputs = new FunnelIOInputsAutoLogged();

    debounceTimestamp = Timer.getFPGATimestamp();
    manipulatorHasCoral = false;

    clapDaddyGoal = FunnelState.OPENED;
    rollerGoal = FunnelRollerState.STOP;

    isClosedLoop = true;
  }

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
      io.setClapDaddyGoal(clapDaddyGoal.getAngle());
    }
    io.setRollerVoltage(rollerGoal.getVoltage());
    InternalLoggedTracer.record("Set Funnel Goal", "Funnel/Periodic");

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
  public void setClapDaddyGoal(FunnelState goal) {
    isClosedLoop = true;
    this.clapDaddyGoal = goal;
  }

  /**
   * Sets the voltage of the roller.
   *
   * @param volts The desired voltage.
   * @return A command to set the roller voltage.
   */
  public void setRollerGoal(FunnelRollerState state) {
    rollerGoal = state;
  }

  /**
   * Runs the SysId routine for the clapDaddy.
   *
   * @return A command to run the SysId routine.
   */
  public Command sysIdRoutine(V2_RedundancySuperstructure superstructure) {
    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(1),
                (state) -> Logger.recordOutput("Funnel/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setClapDaddyVoltage(volts.in(Volts)), null, superstructure));
    return Commands.sequence(
        superstructure.runGoal(V2_RedundancySuperstructureStates.OVERRIDE),
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
