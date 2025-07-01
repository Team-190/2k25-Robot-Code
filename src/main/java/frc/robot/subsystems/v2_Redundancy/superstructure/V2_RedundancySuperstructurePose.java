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
import lombok.Getter;

public class V2_RedundancySuperstructurePose {

  private final String key;

  @Getter private final ReefState elevatorHeight;
  @Getter private final ArmState armState;
  @Getter private final IntakeExtensionState intakeState;
  @Getter private final FunnelState funnelState;

  public V2_RedundancySuperstructurePose(String key, SubsystemPoses poses) {
    this.key = key;

    this.elevatorHeight = poses.elevatorHeight();
    this.armState = poses.armState();
    this.intakeState = poses.intakeState();
    this.funnelState = poses.funnelState();
  }

  public Command setElevatorHeight(V2_RedundancyElevator elevator) {
    return Commands.parallel(
        Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)),
        elevator.waitUntilAtGoal());
  }

  public Command setArmState(V2_RedundancyManipulator manipulator) {
    return Commands.parallel(
        Commands.runOnce(() -> manipulator.setAlgaeArmGoal(armState)),
        manipulator.waitUntilAlgaeArmAtGoal());
  }

  public Command setIntakeState(V2_RedundancyIntake intake) {
    return Commands.parallel(
        Commands.runOnce(() -> intake.setExtensionGoal(intakeState)),
        intake.waitUntilExtensionAtGoal());
  }

  public Command setFunnelState(V2_RedundancyFunnel funnel) {
    return Commands.runOnce(() -> funnel.setClapDaddyGoal(funnelState));
  }

  public Command asCommand(
      V2_RedundancyElevator elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
    return Commands.parallel(
        setElevatorHeight(elevator), setArmState(manipulator),
        setIntakeState(intake), setFunnelState(funnel));
  }

  public String toString() {
    return key;
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
