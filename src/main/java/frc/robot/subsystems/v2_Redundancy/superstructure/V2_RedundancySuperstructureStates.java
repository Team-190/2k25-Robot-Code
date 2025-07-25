package frc.robot.subsystems.v2_Redundancy.superstructure;

import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelRollerState;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureAction.SubsystemActions;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeExtensionState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
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
          ReefState.STOW, ArmState.STOW_UP, IntakeExtensionState.STOW, FunnelState.OPENED),
      SubsystemActions.empty()),
  OVERRIDE("OVERRIDE", new SubsystemPoses(), SubsystemActions.empty()),

  // Coral States - Setpoints
  L1(
      "L1 CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L1, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      SubsystemActions.empty()),
  L2(
      "L2 CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L2, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      SubsystemActions.empty()),
  L3(
      "L3 CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L3, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      SubsystemActions.empty()),
  L4(
      "L4 CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L4, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      SubsystemActions.empty()),
  L4_PLUS(
      "L4+ CORAL SETPOINT",
      new SubsystemPoses(
          ReefState.L4_PLUS, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      SubsystemActions.empty()),

  // Coral States - Actions
  INTAKE_STATION(
      "INTAKE CORAL",
      new SubsystemPoses(),
      new SubsystemActions(
          ManipulatorRollerState.CORAL_INTAKE, FunnelRollerState.INTAKE, IntakeRollerState.STOP)),
  SCORE_L1(
      "L1 CORAL SCORE",
      new SubsystemPoses(
          ReefState.L1, ArmState.STOW_DOWN, IntakeExtensionState.L1_EXT, FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.L1_SCORE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
  SCORE_L2(
      "L2 CORAL SCORE",
      new SubsystemPoses(
          ReefState.L2, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.SCORE_CORAL, FunnelRollerState.STOP, IntakeRollerState.STOP)),
  SCORE_L3(
      "L3 CORAL SCORE",
      new SubsystemPoses(
          ReefState.L3, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.SCORE_CORAL, FunnelRollerState.STOP, IntakeRollerState.STOP)),
  SCORE_L4(
      "L4 CORAL SCORE",
      new SubsystemPoses(
          ReefState.L4, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.L4_SCORE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
  SCORE_L4_PLUS(
      "L4+ CORAL SCORE",
      new SubsystemPoses(
          ReefState.L4_PLUS, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.SCORE_CORAL, FunnelRollerState.STOP, IntakeRollerState.STOP)),

  // Algae States - Floor Operations
  FLOOR_ACQUISITION(
      "FLOOR ALGAE SETPOINT",
      new SubsystemPoses(
          ReefState.ALGAE_FLOOR_INTAKE,
          ArmState.FLOOR_INTAKE,
          IntakeExtensionState.INTAKE,
          FunnelState.OPENED),
      SubsystemActions.empty()),
  INTAKE_FLOOR(
      "INTAKE FLOOR",
      new SubsystemPoses(
          ReefState.ALGAE_FLOOR_INTAKE,
          ArmState.FLOOR_INTAKE,
          IntakeExtensionState.INTAKE,
          FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.ALGAE_INTAKE, FunnelRollerState.STOP, IntakeRollerState.INTAKE)),
  // Algae States - Reef Operations
  REEF_ACQUISITION_L2(
      "L2 ALGAE SETPOINT",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM,
          ArmState.STOW_DOWN,
          IntakeExtensionState.STOW,
          FunnelState.OPENED),
      SubsystemActions.empty()),
  REEF_ACQUISITION_L3(
      "L3 ALGAE SETPOINT",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP,
          ArmState.STOW_DOWN,
          IntakeExtensionState.STOW,
          FunnelState.OPENED),
      SubsystemActions.empty()),
  INTAKE_REEF_L2(
      "L2 ALGAE INTAKE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM,
          ArmState.REEF_INTAKE,
          IntakeExtensionState.STOW,
          FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.ALGAE_INTAKE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
  INTAKE_REEF_L3(
      "L3 ALGAE INTAKE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP,
          ArmState.REEF_INTAKE,
          IntakeExtensionState.STOW,
          FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.ALGAE_INTAKE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
  DROP_REEF_L2(
      "DROP L2 ALGAE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_BOTTOM,
          ArmState.STOW_DOWN,
          IntakeExtensionState.STOW,
          FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.REMOVE_ALGAE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
  DROP_REEF_L3(
      "DROP L3 ALGAE",
      new SubsystemPoses(
          ReefState.ALGAE_INTAKE_TOP,
          ArmState.STOW_DOWN,
          IntakeExtensionState.STOW,
          FunnelState.OPENED),
      new SubsystemActions(
          ManipulatorRollerState.REMOVE_ALGAE, FunnelRollerState.STOP, IntakeRollerState.STOP)),

  // Algae States - Barge/Processor Operations
  BARGE(
      "BARGE SETPOINT",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, ArmState.STOW_UP, IntakeExtensionState.STOW, FunnelState.CLOSED),
      SubsystemActions.empty()),
  PROCESSOR(
      "PROCESSOR SETPOINT",
      new SubsystemPoses(
          ReefState.STOW, ArmState.PROCESSOR, IntakeExtensionState.STOW, FunnelState.CLOSED),
      SubsystemActions.empty()),
  SCORE_BARGE(
      "SCORE BARGE",
      new SubsystemPoses(
          ReefState.ALGAE_SCORE, ArmState.STOW_UP, IntakeExtensionState.STOW, FunnelState.CLOSED),
      new SubsystemActions(
          ManipulatorRollerState.SCORE_ALGAE, FunnelRollerState.STOP, IntakeRollerState.STOP)),
  SCORE_PROCESSOR(
      "SCORE PROCESSOR",
      new SubsystemPoses(
          ReefState.STOW, ArmState.PROCESSOR, IntakeExtensionState.STOW, FunnelState.CLOSED),
      new SubsystemActions(
          ManipulatorRollerState.SCORE_ALGAE, FunnelRollerState.STOP, IntakeRollerState.STOP)),

  // Utility States
  INTERMEDIATE_WAIT_FOR_ELEVATOR(
      "WAIT FOR ELEVATOR",
      new SubsystemPoses(
          ReefState.ALGAE_MID, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      SubsystemActions.empty()),
  INTERMEDIATE_WAIT_FOR_ARM(
      "WAIT FOR ARM",
      new SubsystemPoses(
          ReefState.ALGAE_MID, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.OPENED),
      SubsystemActions.empty()),
  FUNNEL_CLOSE_WITH_STOW_UP(
      "FUNNEL CLOSE WITH STOW UP",
      new SubsystemPoses(
          ReefState.STOW, ArmState.STOW_UP, IntakeExtensionState.STOW, FunnelState.CLOSED),
      SubsystemActions.empty()),
  FUNNEL_CLOSE_WITH_STOW_DOWN(
      "FUNNEL CLOSE WITH STOW DOWN",
      new SubsystemPoses(
          ReefState.STOW, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.CLOSED),
      SubsystemActions.empty()),

  // Climb State
  CLIMB(
      "CLIMB",
      new SubsystemPoses(
          ReefState.STOW, ArmState.STOW_DOWN, IntakeExtensionState.STOW, FunnelState.CLIMB),
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
