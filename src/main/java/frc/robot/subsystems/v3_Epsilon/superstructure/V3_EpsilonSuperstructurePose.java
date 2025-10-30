package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorArmState;
import java.util.Optional;
import lombok.Getter;

/**
 * Represents a specific pose (configuration) of the superstructure, defining the states of the
 * elevator, manipulator arm, intake, and funnel. This class allows for coordinated control of these
 * subsystems to achieve a desired configuration.
 */
public class V3_EpsilonSuperstructurePose {
  private final String key;

  @Getter private final ReefState elevatorHeight;
  @Getter private final ManipulatorArmState armState;
  @Getter private final IntakePivotState intakeState;

  /**
   * Constructs a new V3_EpsilonSuperstructurePose with the given subsystem poses.
   *
   * @param key A unique identifier for this pose.
   * @param poses The combined poses for all relevant subsystems.
   */
  public V3_EpsilonSuperstructurePose(String key, SubsystemPoses poses) {
    this.key = key;

    this.elevatorHeight = poses.elevatorHeight();
    this.armState = poses.manipulatorArmState();
    this.intakeState = poses.intakePivotState();
  }

  /**
   * Creates a command to set the elevator to the specified height for this pose.
   *
   * @param elevator The elevator subsystem to control.
   * @return A Command that sets the elevator height and waits until it reaches the goal.
   */
  public Command setElevatorHeight(ElevatorFSM elevator) {
    return Commands.parallel(Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)));
  }

  /**
   * Creates a command to set the intake to the specified extension state for this pose.
   *
   * @param intake The intake subsystem to control.
   * @return A Command that sets the intake extension state and waits until it reaches the goal.
   */
  public Command setIntakeState(V3_EpsilonIntake intake) {
    return Commands.parallel(Commands.runOnce(() -> intake.setPivotGoal(intakeState)));
  }

  /**
   * Creates a command to set the manipulator arm to the specified state for this pose.
   *
   * @param manipulator The manipulator subsystem to control.
   * @return A Command that sets the arm state and waits until it reaches the goal.
   */
  public Command setManipulatorState(V3_EpsilonManipulator manipulator) {
    return Commands.parallel(Commands.runOnce(() -> manipulator.setArmGoal(armState)));
  }

  /**
   * Creates a command that sets all subsystems (elevator, manipulator, intake, and funnel) to the
   * states defined by this pose.
   *
   * @param elevator The elevator subsystem.
   * @param intake The intake subsystem.
   * @param manipulator The manipulator subsystem.
   * @return A Command that sets all subsystems to their respective states in parallel.
   */
  public Command asConfigurationSpaceCommand(
      ElevatorFSM elevator, V3_EpsilonIntake intake, V3_EpsilonManipulator manipulator) {
    return Commands.parallel(
        Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)),
        Commands.runOnce(() -> manipulator.setArmGoal(armState)),
        Commands.runOnce(() -> intake.setPivotGoal(intakeState)));
  }

  public Command wait(
      ElevatorFSM elevator,
      V3_EpsilonIntake intake,
      V3_EpsilonManipulator manipulator,
      Optional<V3_EpsilonSuperstructureTransitionCondition> condition) {

    return Commands.either(
        Commands.either(
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilArmAtGoal(),
            () ->
                condition
                    .get()
                    .equals(V3_EpsilonSuperstructureTransitionCondition.ELEVATOR_AT_GOAL)),
        Commands.parallel(
            elevator.waitUntilAtGoal(),
            manipulator.waitUntilArmAtGoal(),
            intake.waitUntilPivotAtGoal()),
        () -> condition.isPresent());

    // return Commands.sequence(
    // Commands.parallel(
    // elevator.waitUntilAtGoal(),
    // manipulator.waitUntilArmAtGoal(),
    // intake.waitUntilPivotAtGoal()),
    // Commands.either(
    // Commands.either(
    // elevator.waitUntilAtGoal(),
    // manipulator.waitUntilArmAtGoal(),
    // () ->
    // condition
    // .get()
    // .equals(V3_EpsilonSuperstructureTransitionCondition.ELEVATOR_AT_GOAL)),
    // Commands.none(),
    // () -> condition.isPresent()));
  }

  /**
   * Returns a string representation of this pose (the key).
   *
   * @return The key of this pose.
   */
  public String toString() {
    return key;
  }

  /**
   * A record that groups the states of the elevator, manipulator arm, intake, and funnel
   * subsystems. This is used to define a complete pose for the superstructure.
   */
  public record SubsystemPoses(
      ReefState elevatorHeight,
      ManipulatorArmState manipulatorArmState,
      IntakePivotState intakePivotState) {

    /**
     * Creates a SubsystemPoses instance with default states (STOW for elevator, arm, and intake).
     */
    public SubsystemPoses() {
      this(ReefState.STOW, ManipulatorArmState.VERTICAL_UP, IntakePivotState.STOW);
    }
  }
}
