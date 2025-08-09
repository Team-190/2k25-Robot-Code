package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import lombok.Getter;

public class V3_SuperstructurePoses {
  private String key;

  @Getter private final ReefState elevatorHeight;
  @Getter private final V3_EpsilonManipulatorConstants.PivotState armState;
  @Getter private final V3_EpsilonIntakeConstants.IntakeState intakeState;

  public V3_SuperstructurePoses(String key, SubsystemPoses poses) {
    this.key = key;

    this.elevatorHeight = poses.elevatorHeight();
    this.armState = poses.manipulatorArmState();
    this.intakeState = poses.intakeState();
  }

  public Command setElevatorHeight(ElevatorFSM elevator) {
    return Commands.parallel(
        Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)),
        elevator.waitUntilAtGoal());
  }

  public Command setIntakeState(V3_EpsilonIntake intake) {
    return Commands.parallel(
        Commands.runOnce(()->intake.setGoal(intakeState)), intake.waitUntilPivotAtGoal());
  }

  public Command setManipulatorState(V3_EpsilonManipulator manipulator) {
    return Commands.parallel(
        Commands.runOnce(() -> manipulator.setManipulatorState(armState)),
        manipulator.waitUntilPivotAtGoal());
  }

  public Command asCommand(
      ElevatorFSM elevator, V3_EpsilonManipulator manipulator, V3_EpsilonIntake intake) {
    return Commands.parallel(
        setElevatorHeight(elevator), setManipulatorState(manipulator), setIntakeState(intake));
  }

  public String toString() {
    return key;
  }

  public record SubsystemPoses(
      ReefState elevatorHeight,
      V3_EpsilonManipulatorConstants.PivotState manipulatorArmState,
      V3_EpsilonIntakeConstants.IntakeState intakeState) {

    /**
     * Creates a SubsystemPoses instance with default states (STOW for elevator, arm, and intake;
     * OPENED for funnel).
     */
    public SubsystemPoses() {
      this(
          ReefState.STOW,
          V3_EpsilonManipulatorConstants.PivotState.STOW_DOWN,
          V3_EpsilonIntakeConstants.IntakeState.STOW);
    }
  }
}
