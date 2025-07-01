package frc.robot.subsystems.v2_Redundancy.superstructure;

import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
import lombok.Getter;

public class V2_RedundancySuperstructureAction {
  @Getter private final ManipulatorRollerState manipulatorRollerState;
  @Getter private final FunnelRollerState funnelRollerState;
  @Getter private final IntakeRollerState intakeRollerState;

  public V2_RedundancySuperstructureAction(String key, RollerStates rollerStates) {
    this.manipulatorRollerState = rollerStates.manipulatorRollerState();
    this.funnelRollerState = rollerStates.funnelRollerState();
    this.intakeRollerState = rollerStates.intakeRollerState();
  }

  public void runManipulator(V2_RedundancyManipulator manipulator) {
    manipulator.runManipulator(manipulatorRollerState);
  }

  public void runFunnel(V2_RedundancyFunnel funnel) {
    funnel.setRollerGoal(funnelRollerState);
  }

  public void runIntake(V2_RedundancyIntake intake) {
    intake.setRollerGoal(intakeRollerState);
  }

  public void get(
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
    runManipulator(manipulator);
    runFunnel(funnel);
    runIntake(intake);
  }

  public record RollerStates(
      ManipulatorRollerState manipulatorRollerState,
      FunnelRollerState funnelRollerState,
      IntakeRollerState intakeRollerState) {
    public static RollerStates empty() {
      return new RollerStates(
          ManipulatorRollerState.STOP, FunnelRollerState.STOP, IntakeRollerState.STOP);
    }
  }
}
