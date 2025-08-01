package frc.robot.subsystems.v2_Redundancy.superstructure;

import frc.robot.subsystems.shared.funnel.Funnel.FunnelFSM;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
import lombok.Getter;

/**
 * Coordinates and executes roller actions across multiple subsystems. This class manages the states
 * of manipulator, funnel, and intake rollers and provides methods to control them in a coordinated
 * manner.
 */
public class V2_RedundancySuperstructureAction {
  @Getter private final FunnelRollerState funnelRollerState;
  @Getter private final IntakeRollerState intakeRollerState;
  @Getter private final ManipulatorRollerState manipulatorRollerState;

  /**
   * Creates a new SuperstructureAction with specified roller states.
   *
   * @param key Identifier for the action set
   * @param rollerStates Combined states for all subsystem rollers
   */
  public V2_RedundancySuperstructureAction(String key, SubsystemActions rollerStates) {
    this.funnelRollerState = rollerStates.funnelRollerState();
    this.intakeRollerState = rollerStates.intakeRollerState();
    this.manipulatorRollerState = rollerStates.manipulatorRollerState();
  }

  /**
   * Sets the manipulator roller to its configured state.
   *
   * @param manipulator The manipulator subsystem to control
   */
  public void runManipulator(V2_RedundancyManipulator manipulator) {
    manipulator.setRollerGoal(manipulatorRollerState);
  }

  /**
   * Sets the funnel roller to its configured state.
   *
   * @param funnel The funnel subsystem to control
   */
  public void runFunnel(FunnelFSM funnel) {
    funnel.setRollerGoal(funnelRollerState);
  }

  /**
   * Sets the intake roller to its configured state.
   *
   * @param intake The intake subsystem to control
   */
  public void runIntake(V2_RedundancyIntake intake) {
    intake.setRollerGoal(intakeRollerState);
  }

  /**
   * Executes all roller actions simultaneously across all subsystems.
   *
   * @param funnel The funnel subsystem
   * @param intake The intake subsystem
   * @param manipulator The manipulator subsystem
   */
  public void get(
      FunnelFSM funnel, V2_RedundancyIntake intake, V2_RedundancyManipulator manipulator) {
    runFunnel(funnel);
    runIntake(intake);
    runManipulator(manipulator);
  }

  /**
   * Record that holds the states for all roller subsystems. Provides a way to group and manage
   * multiple roller states as a single unit.
   */
  public record SubsystemActions(
      FunnelRollerState funnelRollerState,
      IntakeRollerState intakeRollerState,
      ManipulatorRollerState manipulatorRollerState) {
    /**
     * Creates a SubsystemActions instance with all rollers in STOP state.
     *
     * @return A new SubsystemActions with all states set to STOP
     */
    public static SubsystemActions empty() {
      return new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.STOP);
    }
  }
}
