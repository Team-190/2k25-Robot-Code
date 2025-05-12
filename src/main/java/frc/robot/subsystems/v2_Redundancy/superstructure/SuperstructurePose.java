package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;

public class SuperstructurePose extends SuperstructureState {

  private final ReefState elevatorHeight;
  private final ArmState armState;
  private final IntakeState intakeState;
  private final FunnelState funnelState;

    public SuperstructurePose(
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

  public IntakeState getIntakeState() {
    return intakeState;
  }

  public FunnelState getFunnelState() {
    return funnelState;
  }

  public Command setElevatorHeight() {
    return elevator.setPosition(() -> elevatorHeight);
  }

  public Command setArmState() {
    return manipulator.setAlgaeArmGoal(armState);
  }

  public Command setIntakeState() {
    return intake.setExtensionGoal(intakeState);
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
      IntakeState intakeState,
      FunnelState funnelState) {

    public SubsystemPoses() {
      this(ReefState.STOW, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED);
    }
  }
}
