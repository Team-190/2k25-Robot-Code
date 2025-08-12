package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import lombok.Getter;

public class V3_SuperstructurePoses {
  private String key;

  @Getter private final ReefState elevatorHeight;
  @Getter private final V3_EpsilonManipulatorConstants.PivotState pivotState;
  @Getter private final V3_EpsilonIntakeConstants.IntakeState intakeState;

  public V3_SuperstructurePoses(String key, SubsystemPoses poses) {
    this.key = key;

    this.elevatorHeight = poses.elevatorHeight();
    this.pivotState = poses.manipulatorArmState();
    this.intakeState = poses.intakeState();
  }

  /**
   * Creates a command to set the elevator to the specified height for this pose.
   *
   * @param elevator
   * @return
   */
  public Command setElevatorHeight(ElevatorFSM elevator) {
    return Commands.parallel(
        Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)),
        elevator.waitUntilAtGoal());
  }

  /**
   * Creates a command to set the intake to the specified state for this pose.
   *
   * @param intake
   * @return
   */
  public Command setIntakeState(V3_EpsilonIntake intake) {
    return Commands.parallel(
        Commands.runOnce(() -> intake.setGoal(intakeState)), intake.waitUntilPivotAtGoal());
  }

  /**
   * Creates a command to set the manipulator arm to the specified state for this pose. This command
   * will also wait until the manipulator arm reaches its goal position.
   *
   * @param manipulator
   * @return
   */
  public Command setManipulatorState(V3_EpsilonManipulator manipulator) {
    return Commands.parallel(
        Commands.runOnce(() -> manipulator.setManipulatorState(pivotState)),
        manipulator.waitUntilPivotAtGoal());
  }

  /**
   * Creates a command that sets the elevator, manipulator, and intake to their respective states
   * defined in this pose. This command runs all three subsystem commands in parallel.
   *
   * @param elevator
   * @param manipulator
   * @param intake
   * @return
   */
  public Command asCommand(
      ElevatorFSM elevator, V3_EpsilonManipulator manipulator, V3_EpsilonIntake intake) {
    return Commands.parallel(
        setElevatorHeight(elevator), setManipulatorState(manipulator), setIntakeState(intake));
  }

  /**
   * Returns a string representation of this pose, which is simply the key. This is useful for
   * debugging and logging purposes.
   *
   * @return A string representation of the pose.
   */
  public String toString() {
    return key;
  }

  /**
   * A record that holds the states of the subsystems (elevator, manipulator arm, and intake) for a
   * specific superstructure pose. This record is used to encapsulate the states of the subsystems
   * in a single object, making it easier to manage and pass around.
   */
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
