package frc.robot.subsystems.v3_Epsilon.superstructure;

import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorRollerState;
import lombok.Getter;

/**
 * Coordinates and executes roller actions across multiple subsystems. This class manages the states
 * of manipulator, funnel, and intake rollers and provides methods to control them in a coordinated
 * manner.
 */
public class V3_EpsilonSuperstructureAction {
  @Getter private final IntakeRollerState intakeRollerState;
  @Getter private final ManipulatorRollerState manipulatorRollerState;

  /**
   * Creates a new SuperstructureAction with specified roller states.
   *
   * @param key Identifier for the action set
   * @param rollerStates Combined states for all subsystem rollers
   */
  public V3_EpsilonSuperstructureAction(String key, SubsystemActions rollerStates) {
    this.intakeRollerState = rollerStates.intakeRollerState();
    this.manipulatorRollerState = rollerStates.manipulatorRollerState();
  }

  /**
   * Sets the manipulator roller to its configured state.
   *
   * @param manipulator The manipulator subsystem to control
   */
  public void runManipulator(V3_EpsilonManipulator manipulator) {
    manipulator.setRollerGoal(manipulatorRollerState);
  }

  /**
   * Sets the intake roller to its configured state.
   *
   * @param intake The intake subsystem to control
   */
  public void runIntake(V3_EpsilonIntake intake) {
    intake.setRollerGoal(intakeRollerState);
  }

  /**
   * Executes all roller actions simultaneously across all subsystems.
   *
   * @param funnel The funnel subsystem
   * @param intake The intake subsystem
   * @param manipulator The manipulator subsystem
   */
  public void get(V3_EpsilonIntake intake, V3_EpsilonManipulator manipulator) {
    runIntake(intake);
    runManipulator(manipulator);
  }

  /**
   * Record that holds the states for all roller subsystems. Provides a way to group and manage
   * multiple roller states as a single unit.
   */
  public record SubsystemActions(
      ManipulatorRollerState manipulatorRollerState, IntakeRollerState intakeRollerState) {
    /**
     * Creates a SubsystemActions instance with all rollers in STOP state.
     *
     * @return A new SubsystemActions with all states set to STOP
     */
    public static SubsystemActions empty() {
      return new SubsystemActions(ManipulatorRollerState.STOP, IntakeRollerState.STOP);
    }
  }
}
