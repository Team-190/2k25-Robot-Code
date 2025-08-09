package frc.robot.subsystems.v3_Epsilon.superstructure;

import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import lombok.Getter;

public class V3_SuperstructureActions {

  @Getter private final V3_EpsilonManipulatorConstants.ManipulatorRollerStates manipulatorRollerState;
  @Getter private final V3_EpsilonIntakeConstants.IntakeRollerStates intakeRollerStates;

  /**
   * Creates a new instance of V3_SuperstructureActions with the specified key and subsystem actions.
   * @param key
   * @param subsystemActions
   */

  public V3_SuperstructureActions(String key, SubsystemActions subsystemActions) {
    this.manipulatorRollerState = V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP;
    this.intakeRollerStates = V3_EpsilonIntakeConstants.IntakeRollerStates.STOP;
  }

  /**
   * Runs the manipulator with the specified roller state.
   * @param manipulator
   */
  public void runManipulator(V3_EpsilonManipulator manipulator) {
    manipulator.setRollerGoal(manipulatorRollerState);
  }

  /**
   * Runs the intake with the specified roller state.
   * @param intake
   */
  public void runIntake(V3_EpsilonIntake intake) {
    intake.setRollerGoal(intakeRollerStates);
  }

  /**
   * Runs both the intake and manipulator simultaneously with their respective roller states.
   * @param intake
   * @param manipulator
   */
  public void get(V3_EpsilonIntake intake, V3_EpsilonManipulator manipulator) {
    // Run all actions simultaneously
    runIntake(intake);
    runManipulator(manipulator);
  }

  /**
   * Represents the actions for the subsystems in the superstructure.
   * This record holds the states for the manipulator roller and intake roller.
   */
  public record SubsystemActions(
      V3_EpsilonManipulatorConstants.ManipulatorRollerStates manipulatorRollerState,
      V3_EpsilonIntakeConstants.IntakeRollerStates intakeRollerStates) {

    public static SubsystemActions empty() {
      return new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP,
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP);
    }
  }
}
