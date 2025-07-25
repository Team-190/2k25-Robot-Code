package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotStateLL;
import frc.robot.subsystems.shared.elevator.ElevatorFSM;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelFSM;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeRollerState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ManipulatorRollerState;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;

/**
 * Manages the state transition graph for the V2 Redundancy Superstructure. This class defines all
 * possible transitions between different states of the robot's superstructure and the commands
 * required to execute these transitions.
 */
public class V2_RedundancySuperstructureEdges {

  public static final ArrayList<Edge> NoneEdges = new ArrayList<>();
  public static final ArrayList<Edge> AlgaeEdges = new ArrayList<>();
  public static final ArrayList<Edge> NoAlgaeEdges = new ArrayList<>();

  /**
   * Represents a directional edge between two superstructure states. Used to define valid
   * transitions in the state machine.
   */
  public record Edge(V2_RedundancySuperstructureStates from, V2_RedundancySuperstructureStates to) {
    public Edge(V2_RedundancySuperstructureStates from, V2_RedundancySuperstructureStates to) {
      this.from = from;
      this.to = to;
    }

    @Override
    public String toString() {
      return from + " -> " + to;
    }
  }

  /**
   * Defines the algae handling capability required for a state transition. NONE - Transition
   * doesn't involve algae handling NO_ALGAE - Transition requires algae handling to be disabled
   * ALGAE - Transition requires algae handling to be enabled
   */
  public enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }

  /**
   * Represents a command associated with an edge in the state transition graph. Extends DefaultEdge
   * to work with JGraphT library while adding custom properties.
   */
  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  }

  /**
   * Gets the command to execute for a given edge in the superstructure state graph. This command
   * typically involves coordinating the elevator, manipulator, funnel, and intake subsystems to
   * move from one state to another.
   *
   * @param from The starting state of the superstructure.
   * @param to The target state of the superstructure.
   * @param elevator The elevator subsystem.
   * @param manipulator The manipulator subsystem.
   * @param funnel The funnel subsystem.
   * @param intake The intake subsystem.
   * @return A {@link Command} that, when executed, transitions the superstructure from the 'from'
   *     state to the 'to' state.
   */
  private static Command getEdgeCommand(
      V2_RedundancySuperstructureStates from,
      V2_RedundancySuperstructureStates to,
      ElevatorFSM elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnelFSM funnel,
      V2_RedundancyIntake intake) {
    V2_RedundancySuperstructurePose pose = to.getPose();

    // Special case: If coming from INTAKE_FLOOR, run outtake for 1s then stop
    if (from == V2_RedundancySuperstructureStates.INTAKE_FLOOR) {
      return Commands.parallel(
          pose.asCommand(elevator, manipulator, funnel, intake),
          Commands.run(() -> intake.setRollerGoal(IntakeRollerState.OUTTAKE))
              .withTimeout(0.75)
              .andThen(Commands.runOnce(() -> intake.setRollerGoal(IntakeRollerState.STOP)))
              .unless(() -> RobotStateLL.isHasAlgae()));
    }

    // Special case: If going to INTAKE_REEF_L2 or INTAKE_REEF_L3, but not from STOW_UP or BARGE
    if ((to == V2_RedundancySuperstructureStates.INTAKE_REEF_L2
            || to == V2_RedundancySuperstructureStates.INTAKE_REEF_L3)
        && (from != V2_RedundancySuperstructureStates.STOW_UP
            || from != V2_RedundancySuperstructureStates.BARGE)) {
      return Commands.sequence(
          pose.setElevatorHeight(elevator)
              .alongWith(
                  Commands.runOnce(() -> manipulator.setAlgaeArmGoal(ArmState.STOW_DOWN))
                      .alongWith(manipulator.waitUntilAlgaeArmAtGoal()))
              .alongWith(pose.setFunnelState(funnel).alongWith(pose.setIntakeState(intake))),
          pose.setArmState(manipulator));
    }

    // Special case: If going to FLOOR_ACQUISITION from any state EXCEPT INTAKE_FLOOR
    if (to == V2_RedundancySuperstructureStates.FLOOR_ACQUISITION)
      return pose.setIntakeState(intake)
          .andThen(
              pose.setElevatorHeight(elevator)
                  .alongWith(pose.setFunnelState(funnel), pose.setArmState(manipulator)));

    // Special case: If going to INTAKE_FLOOR, run intake roller as deadline
    if (to == V2_RedundancySuperstructureStates.INTAKE_FLOOR)
      return Commands.deadline(
          pose.asCommand(elevator, manipulator, funnel, intake),
          Commands.runOnce(() -> intake.setRollerGoal(IntakeRollerState.INTAKE)));

    // Special case: If going from FLOOR_ACQUISITION to STOW_DOWN,
    // or to STOW_UP
    if ((from == V2_RedundancySuperstructureStates.FLOOR_ACQUISITION
            && to == V2_RedundancySuperstructureStates.STOW_DOWN)
        || to == V2_RedundancySuperstructureStates.STOW_UP) {

      return pose.setArmState(manipulator)
          .andThen(
              pose.setIntakeState(intake)
                  .alongWith(pose.setElevatorHeight(elevator), pose.setFunnelState(funnel)));
    }

    // Special case: If going to INTERMEDIATE_WAIT_FOR_ARM
    if (to == V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM) {

      return Commands.deadline(pose.setArmState(manipulator), pose.setElevatorHeight(elevator));
    }

    // Special case: If transitioning to INTERMEDIATE_WAIT_FOR_ELEVATOR
    if (to == V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR) {
      return pose.setElevatorHeight(elevator);
    }

    // Special case: If transitioning from L1 to SCORE_L1, extend the intake
    if (from == V2_RedundancySuperstructureStates.L1
        && to == V2_RedundancySuperstructureStates.SCORE_L1) {
      return Commands.runOnce(() -> manipulator.setRollerGoal(ManipulatorRollerState.L1_SCORE))
          .andThen(
              Commands.waitSeconds(0.05), pose.asCommand(elevator, manipulator, funnel, intake));
    }

    // Special case: If climbing, wait for elevator first
    if (to == V2_RedundancySuperstructureStates.CLIMB) {
      return pose.setElevatorHeight(elevator)
          .andThen(pose.asCommand(elevator, manipulator, funnel, intake));
    }

    // Special case: If scoring, wait for elevator
    if (to.toString().contains("SCORE")
        || List.of(
                V2_RedundancySuperstructureStates.INTAKE_FLOOR,
                V2_RedundancySuperstructureStates.INTAKE_REEF_L2,
                V2_RedundancySuperstructureStates.INTAKE_REEF_L3,
                V2_RedundancySuperstructureStates.DROP_REEF_L2,
                V2_RedundancySuperstructureStates.DROP_REEF_L3,
                V2_RedundancySuperstructureStates.INTAKE_STATION)
            .contains(to)) {
      return Commands.parallel(
          pose.setElevatorHeight(elevator),
          pose.setArmState(manipulator),
          pose.setIntakeState(intake),
          pose.setFunnelState(funnel));
    }

    // Default case: Execute all subsystem poses in parallel
    return pose.asCommand(elevator, manipulator, funnel, intake);
  }

  /**
   * Creates edges for the state transition graph. This method defines all possible transitions
   * between states and categorizes them based on algae handling. It uses the `Edge` record and adds
   * them to the relevant lists (`NoneEdges`, `AlgaeEdges`, `NoAlgaeEdges`).
   */
  private static void createEdges() {

    // CORAL-RELATED STATES
    // Define the coral levels (L1 to L4) for transitions
    List<V2_RedundancySuperstructureStates> coralLevels =
        List.of(
            V2_RedundancySuperstructureStates.L1,
            V2_RedundancySuperstructureStates.L2,
            V2_RedundancySuperstructureStates.L3,
            V2_RedundancySuperstructureStates.L4);

    // stow_down <-> each coral level (bidirectional, no algae)
    for (V2_RedundancySuperstructureStates level : coralLevels) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.STOW_DOWN, level));
      NoAlgaeEdges.add(new Edge(level, V2_RedundancySuperstructureStates.STOW_DOWN));
    }

    // every coral level <-> every other coral level (no algae)
    for (V2_RedundancySuperstructureStates from : coralLevels) {
      for (V2_RedundancySuperstructureStates to : coralLevels) {
        if (from != to) {
          NoneEdges.add(new Edge(from, to));
        }
      }
    }

    // Miscellaneous L4+ transitions
    NoneEdges.add(
        new Edge(V2_RedundancySuperstructureStates.L4_PLUS, V2_RedundancySuperstructureStates.L4));
    NoneEdges.add(
        new Edge(V2_RedundancySuperstructureStates.L4, V2_RedundancySuperstructureStates.L4_PLUS));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.L4_PLUS,
            V2_RedundancySuperstructureStates.STOW_DOWN));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            V2_RedundancySuperstructureStates.L4_PLUS));

    // Map each coral level to its scoring state and create bidirectional transitions (no algae)
    Map<V2_RedundancySuperstructureStates, V2_RedundancySuperstructureStates> coralScoreMap =
        Map.of(
            V2_RedundancySuperstructureStates.L1, V2_RedundancySuperstructureStates.SCORE_L1,
            V2_RedundancySuperstructureStates.L2, V2_RedundancySuperstructureStates.SCORE_L2,
            V2_RedundancySuperstructureStates.L3, V2_RedundancySuperstructureStates.SCORE_L3,
            V2_RedundancySuperstructureStates.L4, V2_RedundancySuperstructureStates.SCORE_L4,
            V2_RedundancySuperstructureStates.L4_PLUS,
                V2_RedundancySuperstructureStates.SCORE_L4_PLUS);
    coralScoreMap.forEach(
        (level, score) -> {
          NoAlgaeEdges.add(new Edge(score, level));
          NoAlgaeEdges.add(new Edge(level, score));
        });

    // Create one-way transitions from each coral level to FLOOR_ACQUISITION and
    // INTERMEDIATE_WAIT_FOR_ELEVATOR
    for (V2_RedundancySuperstructureStates level :
        List.of(V2_RedundancySuperstructureStates.L1, V2_RedundancySuperstructureStates.L2)) {
      for (V2_RedundancySuperstructureStates target :
          List.of(
              V2_RedundancySuperstructureStates.FLOOR_ACQUISITION,
              V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR)) {
        NoneEdges.add(new Edge(level, target));
      }
    }

    // Create one-way transitions for higher coral levels to reef-related states
    for (V2_RedundancySuperstructureStates level :
        List.of(V2_RedundancySuperstructureStates.L3, V2_RedundancySuperstructureStates.L4)) {
      for (V2_RedundancySuperstructureStates target :
          List.of(
              V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
              V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3,
              V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM)) {
        NoneEdges.add(new Edge(level, target));
      }
    }

    // Create bidirectional transitions between INTAKE_CORAL and STOW_DOWN (no algae)
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.INTAKE_STATION));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_STATION,
            V2_RedundancySuperstructureStates.STOW_DOWN));

    // Create one-way transitions from INTERMEDIATE_WAIT_FOR_ELEVATOR to multiple destinations
    List<V2_RedundancySuperstructureStates> iveDestinations =
        List.of(
            V2_RedundancySuperstructureStates.STOW_UP,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3,
            V2_RedundancySuperstructureStates.BARGE,
            V2_RedundancySuperstructureStates.PROCESSOR);
    for (V2_RedundancySuperstructureStates dest : iveDestinations) {
      AlgaeEdges.add(
          new Edge(V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR, dest));
    }

    // Create one-way transitions from INTERMEDIATE_WAIT_FOR_ARM to multiple destinations
    List<V2_RedundancySuperstructureStates> iwaDestinations =
        List.of(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.L1,
            V2_RedundancySuperstructureStates.L2,
            V2_RedundancySuperstructureStates.FLOOR_ACQUISITION);
    for (V2_RedundancySuperstructureStates dest : iwaDestinations) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM, dest));
    }

    // Create bidirectional transitions between STOW_UP and multiple destinations (algae handling)
    List<V2_RedundancySuperstructureStates> stowUpDestinations =
        List.of(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            V2_RedundancySuperstructureStates.BARGE,
            V2_RedundancySuperstructureStates.PROCESSOR);
    for (V2_RedundancySuperstructureStates dest : stowUpDestinations) {
      AlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.STOW_UP, dest));
      AlgaeEdges.add(new Edge(dest, V2_RedundancySuperstructureStates.STOW_UP));
    }

    // Create one-way transitions from FLOOR_ACQUISITION to multiple destinations (no algae
    // handling)
    List<V2_RedundancySuperstructureStates> floorAcqDest =
        List.of(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.INTAKE_FLOOR);
    for (V2_RedundancySuperstructureStates dest : floorAcqDest) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.FLOOR_ACQUISITION, dest));
    }
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.FLOOR_ACQUISITION,
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR));

    // Define reef-related acquisition states and their transitions
    Map<V2_RedundancySuperstructureStates, List<V2_RedundancySuperstructureStates>> reefMap =
        Map.of(
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
                List.of(
                    V2_RedundancySuperstructureStates.INTAKE_REEF_L2, // no alg
                    V2_RedundancySuperstructureStates.DROP_REEF_L2, // no alg
                    V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM, // none
                    V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3, // none
                    V2_RedundancySuperstructureStates.BARGE, // alg
                    V2_RedundancySuperstructureStates.PROCESSOR, // alg
                    V2_RedundancySuperstructureStates.STOW_UP), // alg
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3,
                List.of(
                    V2_RedundancySuperstructureStates.INTAKE_REEF_L3,
                    V2_RedundancySuperstructureStates.DROP_REEF_L3,
                    V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
                    V2_RedundancySuperstructureStates.BARGE,
                    V2_RedundancySuperstructureStates.PROCESSOR,
                    V2_RedundancySuperstructureStates.STOW_UP));
    reefMap.forEach(
        (from, targets) -> {
          for (V2_RedundancySuperstructureStates to : targets) {
            // forward edge can vary
            if (List.of(
                    V2_RedundancySuperstructureStates.INTAKE_REEF_L2,
                    V2_RedundancySuperstructureStates.DROP_REEF_L2,
                    V2_RedundancySuperstructureStates.INTAKE_REEF_L3,
                    V2_RedundancySuperstructureStates.DROP_REEF_L3)
                .contains(to)) {
              NoAlgaeEdges.add(new Edge(from, to));
            } else if (List.of(
                    V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
                    V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3)
                .contains(to)) {
              NoneEdges.add(new Edge(from, to));
            } else {
              AlgaeEdges.add(new Edge(from, to));
            }
            NoneEdges.add(new Edge(to, from)); // Reverse edge uses NONE
          }
        });

    // Create transitions for BARGE and PROCESSOR states
    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.PROCESSOR,
            V2_RedundancySuperstructureStates.SCORE_BARGE)) {
      AlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.BARGE, dest));
    }
    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3)) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.BARGE, dest));
    }

    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.SCORE_BARGE,
            V2_RedundancySuperstructureStates.BARGE));

    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.BARGE,
            V2_RedundancySuperstructureStates.SCORE_PROCESSOR)) {
      AlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.PROCESSOR, dest));
    }
    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3)) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.PROCESSOR, dest));
    }
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.SCORE_PROCESSOR,
            V2_RedundancySuperstructureStates.PROCESSOR));

    // Create transitions for FLOOR_INTAKE, REEF_INTAKE, and REEF_DROP states
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_FLOOR,
            V2_RedundancySuperstructureStates.FLOOR_ACQUISITION));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_REEF_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_REEF_L3,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.DROP_REEF_L2,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.DROP_REEF_L3,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3));
    AlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_REEF_L2,
            V2_RedundancySuperstructureStates.DROP_REEF_L2));
    AlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTAKE_REEF_L3,
            V2_RedundancySuperstructureStates.DROP_REEF_L3));

    // Create one-way transition from START to STOW_DOWN
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.START, V2_RedundancySuperstructureStates.STOW_DOWN));

    // Create transitions for STOW_DOWN state
    for (V2_RedundancySuperstructureStates dest :
        List.of(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR,
            V2_RedundancySuperstructureStates.FLOOR_ACQUISITION)) {
      NoAlgaeEdges.add(new Edge(V2_RedundancySuperstructureStates.STOW_DOWN, dest));
    }

    // Create bidirectional transitions between STOW_DOWN and CLIMB
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_DOWN, V2_RedundancySuperstructureStates.CLIMB));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.CLIMB, V2_RedundancySuperstructureStates.STOW_DOWN));

    // Create bidirectional transitions for funnel-related states
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.FUNNEL_CLOSE_WITH_STOW_DOWN));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.FUNNEL_CLOSE_WITH_STOW_DOWN,
            V2_RedundancySuperstructureStates.STOW_DOWN));

    AlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_UP,
            V2_RedundancySuperstructureStates.FUNNEL_CLOSE_WITH_STOW_UP));
    AlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.FUNNEL_CLOSE_WITH_STOW_UP,
            V2_RedundancySuperstructureStates.STOW_UP));

    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2));
    NoneEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.STOW_DOWN,
            V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3));
    NoAlgaeEdges.add(
        new Edge(
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR,
            V2_RedundancySuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM));

    // OVERRIDE should not have transitions so that the robot state does not change
  }

  /**
   * Adds edges to the superstructure state graph based on the provided list of edges and algae
   * condition.
   *
   * @param graph The graph to which edges are added.
   * @param edges A list of {@link Edge} objects representing the transitions between states.
   * @param type The {@link AlgaeEdge} type associated with these edges.
   * @param elevator The elevator subsystem.
   * @param manipulator The manipulator subsystem.
   * @param funnel The funnel subsystem.
   * @param intake The intake subsystem.
   */
  private static void addEdges(
      Graph<V2_RedundancySuperstructureStates, EdgeCommand> graph,
      List<Edge> edges,
      AlgaeEdge type,
      ElevatorFSM elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnelFSM funnel,
      V2_RedundancyIntake intake) {
    // Iterate through each edge in the provided list
    for (Edge edge : edges) {
      // Add the edge to the graph with its associated command and algae type
      graph.addEdge(
          edge.from(),
          edge.to(),
          EdgeCommand.builder()
              .command(
                  getEdgeCommand(edge.from(), edge.to(), elevator, manipulator, funnel, intake))
              .algaeEdgeType(type)
              .build());
    }
  }

  /**
   * Adds all predefined edges to the superstructure state graph, categorized by algae condition.
   *
   * @param graph The graph to which edges are added.
   * @param elevator The elevator subsystem.
   * @param manipulator The manipulator subsystem.
   * @param funnel The funnel subsystem.
   * @param intake The intake subsystem.
   */
  public static void addEdges(
      Graph<V2_RedundancySuperstructureStates, EdgeCommand> graph,
      ElevatorFSM elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnelFSM funnel,
      V2_RedundancyIntake intake) {
    // Create all edge lists (NoneEdges, NoAlgaeEdges, AlgaeEdges)
    createEdges();

    // Add edges to the graph for each algae condition
    addEdges(graph, NoneEdges, AlgaeEdge.NONE, elevator, manipulator, funnel, intake);
    addEdges(graph, NoAlgaeEdges, AlgaeEdge.NO_ALGAE, elevator, manipulator, funnel, intake);
    addEdges(graph, AlgaeEdges, AlgaeEdge.ALGAE, elevator, manipulator, funnel, intake);
  }
}
