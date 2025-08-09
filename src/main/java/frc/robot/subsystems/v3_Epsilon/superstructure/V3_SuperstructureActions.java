package frc.robot.subsystems.v3_Epsilon.superstructure;

import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntake;
import lombok.Getter;

public class V3_SuperstructureActions {
  
  @Getter private final V3_EpsilonManipulatorConstants.ManipulatorRollerStates manipulatorRollerState;
  @Getter private final V3_EpsilonIntakeConstants.IntakeRollerStates intakeRollerStates;

  public V3_SuperstructureActions(String key, SubsystemActions subsystemActions) {
    this.manipulatorRollerState = V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP;
    this.intakeRollerStates = V3_EpsilonIntakeConstants.IntakeRollerStates.STOP;
  }

  public void runManipulator(V3_EpsilonManipulator manipulator) {
    manipulator.setRollerGoal(manipulatorRollerState);
  }

  public void runIntake(V3_EpsilonIntake intake) {
    intake.setRollerGoal(intakeRollerStates);
  }

  public void get(V3_EpsilonIntake intake, V3_EpsilonManipulator manipulator) {
    // Run all actions simultaneously
    runIntake(intake);
    runManipulator(manipulator);
  }
  
  public record SubsystemActions (
      V3_EpsilonManipulatorConstants.ManipulatorRollerStates manipulatorRollerState,
      V3_EpsilonIntakeConstants.IntakeRollerStates intakeRollerStates
  ) {

    public static SubsystemActions empty() {
      return new SubsystemActions(
          V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP,
          V3_EpsilonIntakeConstants.IntakeRollerStates.STOP
      );
    }
  }
}
