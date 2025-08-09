package frc.robot.subsystems.v3_Epsilon.superstructure;

// Adjust the package path as needed
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructureActions.SubsystemActions;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_SuperstructurePoses.SubsystemPoses; // Ensure this is the correct package path

public enum V3_SuperstructureStates {
  START("START", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_DOWN("STOW_DOWN", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_UP("STOW_UP", new SubsystemPoses(), SubsystemActions.empty());

  private final String name;
  private final SubsystemPoses pose;
  private final SubsystemActions action;

  V3_SuperstructureStates(String name, SubsystemPoses pose, SubsystemActions action) {
    this.name = name;
    this.pose = pose;
    this.action = action;
  }
}
