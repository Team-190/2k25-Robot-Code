package frc.robot.subsystems.v3_Epsilon.superstructure;

// Adjust the package path as needed
import frc.robot.FieldConstants;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructureActions.SubsystemActions;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructurePoses.SubsystemPoses; // Ensure this is the correct package path

public enum V3_SuperstructureStates {
  START("START", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_DOWN("STOW_DOWN", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_UP("STOW_UP", new SubsystemPoses(
    FieldConstants.Reef.ReefState.STOW, 
    V3_EpsilonManipulatorConstants.PivotState.STOW_UP, 
    V3_EpsilonIntakeConstants.IntakeState.STOW
  ), SubsystemActions.empty()),

  // Coral States - Setpoints
  L1(
      "L1", 
      new SubsystemPoses(
      FieldConstants.Reef.ReefState.L1, 
      V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE, 
      V3_EpsilonIntakeConstants.IntakeState.L1
      ),
      SubsystemActions.empty()
    ),

  L2(
    "L2", new SubsystemPoses(
    FieldConstants.Reef.ReefState.L2, 
    V3_EpsilonManipulatorConstants.PivotState.PRE_SCORE, 
    V3_EpsilonIntakeConstants.IntakeState.L1
    ),
    SubsystemActions.empty()
  );


  private final String name;
  private final SubsystemPoses pose;
  private final SubsystemActions action;

  V3_SuperstructureStates(String name, SubsystemPoses pose, SubsystemActions action) {
    this.name = name;
    this.pose = pose;
    this.action = action;
  }
}
