package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeExtensionState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;

public class V2_RedundancySuperstructurePose extends V2_RedundancySuperstructureState {

  private final ReefState elevatorHeight;
  private final ArmState armState;
  private final IntakeExtensionState intakeState;
  private final FunnelState funnelState;

  public V2_RedundancySuperstructurePose(
      String key,
      SubsystemPoses poses,
      V2_RedundancyElevator elevator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake) {
    super(key, elevator, manipulator, funnel, intake);
    this.elevatorHeight = poses.elevatorHeight();
    this.armState = poses.armState;
    this.intakeState = poses.intakeState;
    this.funnelState = poses.funnelState;
  }

  public ReefState getElevatorHeight() {
    return elevatorHeight;
  }

  public ArmState getArmState() {
    return armState;
  }

  public IntakeExtensionState getIntakeState() {
    return intakeState;
  }

  public FunnelState getFunnelState() {
    return funnelState;
  }

  public Command setElevatorHeight() {
    return Commands.parallel(
        elevator.setPosition(() -> elevatorHeight), elevator.waitUntilAtGoal());
  }

  public Command setArmState() {
    return Commands.parallel(
        manipulator.setAlgaeArmGoal(armState), manipulator.waitUntilAlgaeArmAtGoal());
  }

  public Command setIntakeState() {
    return Commands.parallel(
        intake.setExtensionGoal(intakeState), intake.waitUntilExtensionAtGoal());
  }

  public Command setFunnelState() {
    return funnel.setClapDaddyGoal(funnelState);
  }

  public Command action() {
    return Commands.parallel(
        setElevatorHeight(), setArmState(),
        setIntakeState(), setFunnelState());
  }

  public record SubsystemPoses(
      ReefState elevatorHeight,
      ArmState armState,
      IntakeExtensionState intakeState,
      FunnelState funnelState) {

    public SubsystemPoses() {
      this(ReefState.STOW, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED);
    }
  }
}
