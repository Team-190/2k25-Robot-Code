package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import java.util.ArrayList;
import java.util.List;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;

public class V3_EpsilonSuperstructureEdges {
  public static final ArrayList<Edge> NONE_EDGES = new ArrayList<>();
  public static final ArrayList<Edge> CORAL_EDGES = new ArrayList<>();
  public static final ArrayList<Edge> ALGAE_EDGES = new ArrayList<>();

  public record Edge(V3_EpsilonSuperstructureStates from, V3_EpsilonSuperstructureStates to) {
    public Edge(V3_EpsilonSuperstructureStates from, V3_EpsilonSuperstructureStates to) {
      this.from = from;
      this.to = to;
    }

    @Override
    public String toString() {
      return from + " -> " + to;
    }
  }

  public enum GamePieceEdge {
    NONE,
    CORAL,
    ALGAE
  }

  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final GamePieceEdge gamePieceEdge = GamePieceEdge.NONE;
  }

  /**
   * Gets the command to execute for a given edge in the superstructure state graph. This command
   * typically involves coordinating the elevator, funnel, intake, and manipulator subsystems to
   * move from one state to another.
   *
   * @param from The starting state of the superstructure.
   * @param to The target state of the superstructure.
   * @param elevator The elevator subsystem.
   * @param intake The intake subsystem.
   * @param manipulator The manipulator subsystem.
   * @return A {@link Command} that, when executed, transitions the superstructure from the 'from'
   *     state to the 'to' state.
   */
  private static Command getEdgeCommand(
      V3_EpsilonSuperstructureStates from,
      V3_EpsilonSuperstructureStates to,
      ElevatorFSM elevator,
      V3_EpsilonIntake intake,
      V3_EpsilonManipulator manipulator) {
    V3_EpsilonSuperstructurePose pose = to.getPose();
    // Default case: Execute all subsystem poses in parallel
    return pose.asCommand(elevator, intake, manipulator);
  }

  private static void createEdges() {
  }

  /**
   * Adds edges to the superstructure state graph based on the provided list of edges and algae
   * condition.
   *
   * @param graph The graph to which edges are added.
   * @param edges A list of {@link Edge} objects representing the transitions between states.
   * @param type The {@link GamePieceEdge} type associated with these edges.
   * @param elevator The elevator subsystem.
   * @param manipulator The manipulator subsystem.
   * @param funnel The funnel subsystem.
   * @param intake The intake subsystem.
   */
  private static void addEdges(
      Graph<V3_EpsilonSuperstructureStates, EdgeCommand> graph,
      List<Edge> edges,
      GamePieceEdge type,
      ElevatorFSM elevator,
      V3_EpsilonManipulator manipulator,
      V3_EpsilonIntake intake) {
    // Iterate through each edge in the provided list
    for (Edge edge : edges) {
      // Add the edge to the graph with its associated command and algae type
      graph.addEdge(
          edge.from(),
          edge.to(),
          EdgeCommand.builder()
              .command(getEdgeCommand(edge.from(), edge.to(), elevator, intake, manipulator))
              .gamePieceEdge(type)
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
      Graph<V3_EpsilonSuperstructureStates, EdgeCommand> graph,
      ElevatorFSM elevator,
      V3_EpsilonIntake intake,
      V3_EpsilonManipulator manipulator) {
    // Create all edge lists (NoneEdges, NoAlgaeEdges, AlgaeEdges)
    createEdges();

    // Add edges to the graph for each algae condition
    addEdges(graph, NONE_EDGES, GamePieceEdge.NONE, elevator, manipulator, intake);
    addEdges(graph, CORAL_EDGES, GamePieceEdge.CORAL, elevator, manipulator, intake);
    addEdges(graph, ALGAE_EDGES, GamePieceEdge.ALGAE, elevator, manipulator, intake);
  }
}
