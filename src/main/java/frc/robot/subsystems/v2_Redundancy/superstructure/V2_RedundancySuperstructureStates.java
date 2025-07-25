package frc.robot.subsystems.v2_Redundancy.superstructure;

import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelRollerState;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureAction.SubsystemActions;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeExtensionState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorArmState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;

/**
 * Represents all possible states of the robot's superstructure system. States are organized into
 * logical groups: - Basic States: Default system positions (START, STOW, OVERRIDE) - Coral States:
 * Positions and actions for coral game pieces - Algae States: Floor, reef, and barge/processor
 * operations - Utility States: Intermediate positions and special functions
 */
public enum V2_RedundancySuperstructureStates {
  // Basic States
  START("START", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_DOWN("STOW DOWN", new SubsystemPoses(), SubsystemActions.empty()),
  STOW_UP(
      "STOW UP",
      new SubsystemPoses(
          ReefState.STOW,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_UP),
      SubsystemActions.empty()),
  OVERRIDE("OVERRIDE", new SubsystemPoses(), SubsystemActions.empty()),

  // Coral States - Setpoints
  L1(
      "L1 CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L1,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  L2(
      "L2 CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L2,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  L3(
      "L3 CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L3,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  L4(
      "L4 CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L4,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  L4_PLUS(
      "L4+ CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L4_PLUS,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),

  // Coral States - Actions
  INTAKE_STATION(
      "INTAKE CORAL",
      new SubsystemPoses(),
      new SubsystemActions(
          FunnelRollerState.INTAKE, IntakeRollerState.STOP, ManipulatorRollerState.CORAL_INTAKE)),
  SCORE_L1(
      "L1 CORAL SCORE",
      new SubsystemPoses(
          ReefState.L1,
          FunnelState.OPENED,
          IntakeExtensionState.L1_EXT,
          ManipulatorArmState.STOW_DOWN),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.L1_SCORE)),
  SCORE_L2(
      "L2 CORAL SCORE",
      new SubsystemPoses(
          ReefState.L2,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.SCORE_CORAL)),
  SCORE_L3(
      "L3 CORAL SCORE",
      new SubsystemPoses(
          ReefState.L3,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.SCORE_CORAL)),
  SCORE_L4(
      "L4 CORAL SCORE",
      new SubsystemPoses(
          ReefState.L4,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.L4_SCORE)),
  SCORE_L4_PLUS(
      "L4+ CORAL SCORE",
      new SubsystemPoses(
          ReefState.L4_PLUS,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.SCORE_CORAL)),

  // Algae States - Floor Operations
  FLOOR_ACQUISITION(
      "FLOOR ALGAE SETPOINT",
      new SubsystemPoses(
          ReefState.ALGAE_FLOOR_INTAKE,
          FunnelState.OPENED,
          IntakeExtensionState.INTAKE,
          ManipulatorArmState.FLOOR_INTAKE),
      SubsystemActions.empty()),
  INTAKE_FLOOR(
      "INTAKE FLOOR",
      new SubsystemPoses(
          ReefState.ALGAE_FLOOR_INTAKE,
          FunnelState.OPENED,
          IntakeExtensionState.INTAKE,
          ManipulatorArmState.FLOOR_INTAKE),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.INTAKE, ManipulatorRollerState.ALGAE_INTAKE)),

  // Algae States - Reef Operations
  REEF_ACQUISITION_L2(
      "L2 ALGAE SETPOINT",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  REEF_ACQUISITION_L3(
      "L3 ALGAE SETPOINT",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  INTAKE_REEF_L2(
      "L2 ALGAE INTAKE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.REEF_INTAKE),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.ALGAE_INTAKE)),
  INTAKE_REEF_L3(
      "L3 ALGAE INTAKE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.REEF_INTAKE),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.ALGAE_INTAKE)),
  DROP_REEF_L2(
      "DROP L2 ALGAE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.REMOVE_ALGAE)),
  DROP_REEF_L3(
      "DROP L3 ALGAE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.REMOVE_ALGAE)),

  // Algae States - Barge/Processor Operations
  BARGE(
      "BARGE SETPOINT",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, FunnelState.CLOSED,
          IntakeExtensionState.STOW, ManipulatorArmState.STOW_UP),
      SubsystemActions.empty()),
  PROCESSOR(
      "PROCESSOR SETPOINT",
      new SubsystemPoses(
          ReefState.STOW, FunnelState.CLOSED,
          IntakeExtensionState.STOW, ManipulatorArmState.PROCESSOR),
      SubsystemActions.empty()),
  SCORE_BARGE(
      "SCORE BARGE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, FunnelState.CLOSED,
          IntakeExtensionState.STOW, ManipulatorArmState.STOW_UP),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.SCORE_ALGAE)),
  SCORE_PROCESSOR(
      "SCORE PROCESSOR",
      new SubsystemPoses(
          ReefState.STOW, FunnelState.CLOSED,
          IntakeExtensionState.STOW, ManipulatorArmState.PROCESSOR),
      new SubsystemActions(
          FunnelRollerState.STOP, IntakeRollerState.STOP, ManipulatorRollerState.SCORE_ALGAE)),

  // Utility States
  INTERMEDIATE_WAIT_FOR_ELEVATOR(
      "WAIT FOR ELEVATOR",
      new SubsystemPoses(
          ReefState.ALGAE_MID,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  INTERMEDIATE_WAIT_FOR_ARM(
      "WAIT FOR ARM",
      new SubsystemPoses(
          ReefState.ALGAE_MID,
          FunnelState.OPENED,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  FUNNEL_CLOSE_WITH_STOW_UP(
      "FUNNEL CLOSE WITH STOW UP",
      new SubsystemPoses(
          ReefState.STOW, FunnelState.CLOSED,
          IntakeExtensionState.STOW, ManipulatorArmState.STOW_UP),
      SubsystemActions.empty()),
  FUNNEL_CLOSE_WITH_STOW_DOWN(
      "FUNNEL CLOSE WITH STOW DOWN",
      new SubsystemPoses(
          ReefState.STOW, FunnelState.CLOSED,
          IntakeExtensionState.STOW, ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),

  // Climb State
  CLIMB(
      "CLIMB",
      new SubsystemPoses(
          ReefState.STOW,
          FunnelState.CLIMB,
          IntakeExtensionState.STOW,
          ManipulatorArmState.STOW_DOWN),
      SubsystemActions.empty()),
  ;

  /** Human-readable name of the state */
  private final String name;
  /** Target positions for all subsystems in this state */
  private final SubsystemPoses subsystemPoses;
  /** Action commands (like roller speeds) for this state */
  private final SubsystemActions subsystemActions;

  /**
   * Constructs a new superstructure state.
   *
   * @param name The human-readable name of the state
   * @param poses The target positions for all subsystems
   * @param rollerStates The action commands for mechanisms like rollers
   */
  private V2_RedundancySuperstructureStates(
      String name, SubsystemPoses poses, SubsystemActions rollerStates) {
    this.name = name;
    this.subsystemPoses = poses;
    this.subsystemActions = rollerStates;
  }

  /**
   * Gets the position configuration for this state.
   *
   * @return A V2_RedundancySuperstructurePose object containing all subsystem positions
   */
  public V2_RedundancySuperstructurePose getPose() {
    return new V2_RedundancySuperstructurePose(name, subsystemPoses);
  }

  /**
   * Gets the action commands for this state.
   *
   * @return A V2_RedundancySuperstructureAction object containing all subsystem actions
   */
  public V2_RedundancySuperstructureAction getAction() {
    return new V2_RedundancySuperstructureAction(name, subsystemActions);
  }

  /**
   * Returns the name of this state.
   *
   * @return The state's name as a String
   */
  @Override
  public String toString() {
    return name;
  }
}
