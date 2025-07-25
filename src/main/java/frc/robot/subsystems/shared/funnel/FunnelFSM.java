package frc.robot.subsystems.shared.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateLL;
import frc.robot.RobotStateLL.RobotMode;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelRollerState;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates;
import frc.robot.util.ExternalLoggedTracer;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class FunnelFSM extends Funnel {

  @Setter private boolean manipulatorHasCoral;

  @Getter
  @AutoLogOutput(key = "Funnel/Roller Goal")
  private FunnelRollerState rollerGoal;

  public FunnelFSM(FunnelIO io) {
    super(io, new FunnelIOInputsAutoLogged());

    manipulatorHasCoral = false;

    rollerGoal = FunnelRollerState.STOP;
  }

  public void periodic() {
    ExternalLoggedTracer.reset();

    super.periodic();
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
  public Command sysIdRoutine(V2_RedundancySuperstructure subsystem) {
    return subsystem
        .runGoal(V2_RedundancySuperstructureStates.OVERRIDE)
        .andThen(super.sysIdRoutine(subsystem));
  }
}
