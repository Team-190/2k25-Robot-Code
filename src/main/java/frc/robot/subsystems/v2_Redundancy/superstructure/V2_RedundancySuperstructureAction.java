package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
import lombok.Getter;

public class V2_RedundancySuperstructureAction extends V2_RedundancySuperstructureState {
  @Getter private final ManipulatorRollerState manipulatorRollerState;
  @Getter private final FunnelRollerState funnelRollerState;
  @Getter private final IntakeRollerState intakeRollerState;

  public V2_RedundancySuperstructureAction(
      String key,
      RollerStates rollerStates,
      V2_RedundancyElevator elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
    super(key, elevator, manipulator, funnel, intake);
    this.manipulatorRollerState = rollerStates.manipulatorRollerState();
    this.funnelRollerState = rollerStates.funnelRollerState();
    this.intakeRollerState = rollerStates.intakeRollerState();
  }

  public Command runManipulator() {
    return super.manipulator.runManipulator(manipulatorRollerState);
  }

  public Command runFunnel() {
    return super.funnel.setRollerGoal(funnelRollerState);
  }

  public Command runIntake() {
    return super.intake.setRollerGoal(intakeRollerState);
  }

  public Command asCommand() {
    return Commands.parallel(runManipulator(), runFunnel(), runIntake());
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
