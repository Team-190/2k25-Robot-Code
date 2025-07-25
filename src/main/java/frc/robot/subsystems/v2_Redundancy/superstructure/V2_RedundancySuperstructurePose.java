package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.shared.funnel.Funnel.FunnelFSM;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeExtensionState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import lombok.Getter;

/**
 * Represents a specific pose (configuration) of the superstructure, defining the states of the
 * elevator, manipulator arm, intake, and funnel. This class allows for coordinated control of these
 * subsystems to achieve a desired configuration.
 */
public class V2_RedundancySuperstructurePose {

  private final String key;

  @Getter private final ReefState elevatorHeight;
  @Getter private final ArmState armState;
  @Getter private final IntakeExtensionState intakeState;
  @Getter private final FunnelState funnelState;

  /**
   * Constructs a new V2_RedundancySuperstructurePose with the given subsystem poses.
   *
   * @param key A unique identifier for this pose.
   * @param poses The combined poses for all relevant subsystems.
   */
  public V2_RedundancySuperstructurePose(String key, SubsystemPoses poses) {
    this.key = key;

    this.elevatorHeight = poses.elevatorHeight();
    this.armState = poses.armState();
    this.intakeState = poses.intakeState();
    this.funnelState = poses.funnelState();
  }

  /**
   * Creates a command to set the elevator to the specified height for this pose.
   *
   * @param elevator The elevator subsystem to control.
   * @return A Command that sets the elevator height and waits until it reaches the goal.
   */
  public Command setElevatorHeight(ElevatorFSM elevator) {
    return Commands.parallel(
        Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)),
        elevator.waitUntilAtGoal());
  }

  /**
   * Creates a command to set the manipulator arm to the specified state for this pose.
   *
   * @param manipulator The manipulator subsystem to control.
   * @return A Command that sets the arm state and waits until it reaches the goal.
   */
  public Command setArmState(V2_RedundancyManipulator manipulator) {
    return Commands.parallel(
        Commands.runOnce(() -> manipulator.setAlgaeArmGoal(armState)),
        manipulator.waitUntilAlgaeArmAtGoal());
  }

  /**
   * Creates a command to set the intake to the specified extension state for this pose.
   *
   * @param intake The intake subsystem to control.
   * @return A Command that sets the intake extension state and waits until it reaches the goal.
   */
  public Command setIntakeState(V2_RedundancyIntake intake) {
    return Commands.parallel(
        Commands.runOnce(() -> intake.setExtensionGoal(intakeState)),
        intake.waitUntilExtensionAtGoal());
  }

  /**
   * Creates a command to set the funnel to the specified state for this pose.
   *
   * @param funnel The funnel subsystem to control.
   * @return A Command that sets the funnel state.
   */
  public Command setFunnelState(FunnelFSM funnel) {
    return Commands.runOnce(() -> funnel.setClapDaddyGoal(funnelState));
  }

  /**
   * Creates a command that sets all subsystems (elevator, manipulator, intake, and funnel) to the
   * states defined by this pose.
   *
   * @param elevator The elevator subsystem.
   * @param manipulator The manipulator subsystem.
   * @param funnel The funnel subsystem.
   * @param intake The intake subsystem.
   * @return A Command that sets all subsystems to their respective states in parallel.
   */
  public Command asCommand(
      ElevatorFSM elevator,
      V2_RedundancyManipulator manipulator,
      FunnelFSM funnel,
      V2_RedundancyIntake intake) {
    return Commands.parallel(
        Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)),
            Commands.runOnce(() -> manipulator.setAlgaeArmGoal(armState)),
        Commands.runOnce(() -> intake.setExtensionGoal(intakeState)), setFunnelState(funnel));
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
      ArmState armState,
      IntakeExtensionState intakeState,
      FunnelState funnelState) {

    /**
     * Creates a SubsystemPoses instance with default states (STOW for elevator, arm, and intake;
     * OPENED for funnel).
     */
    public SubsystemPoses() {
      this(ReefState.STOW, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED);
    }
  }
}
