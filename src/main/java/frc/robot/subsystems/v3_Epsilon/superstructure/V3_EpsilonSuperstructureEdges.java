package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import java.io.FileReader;
import java.io.Reader;
import java.util.ArrayList;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.nio.dot.DOTImporter;

public class V3_EpsilonSuperstructureEdges {
  public static final ArrayList<Edge> UNCONSTRAINED = new ArrayList<>();
  public static final ArrayList<Edge> NO_ALGAE_EDGES = new ArrayList<>();

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
    UNCONSTRAINED,
    NO_ALGAE
  }

  public static void loadEdgesFromDot(
      String path,
      Graph<V3_EpsilonSuperstructureStates, EdgeCommand> graph,
      ElevatorFSM elevator,
      V3_EpsilonIntake intake,
      V3_EpsilonManipulator manipulator) {

    DOTImporter<V3_EpsilonSuperstructureStates, EdgeCommand> importer = new DOTImporter<>();

    // --- Fix 1: safer vertex factory ---
    importer.setVertexFactory(
        id -> {
          try {
            return V3_EpsilonSuperstructureStates.valueOf(id);
          } catch (IllegalArgumentException e) {
            System.err.println("[DOT Import] Unknown vertex in DOT: " + id);
            return null;
          }
        });

    // --- Fix 2: ensure edge objects are valid ---
    importer.setEdgeWithAttributesFactory(
        attributes -> {
          GamePieceEdge type = GamePieceEdge.UNCONSTRAINED;

          if (attributes.containsKey("type")) {
            try {
              type = GamePieceEdge.valueOf(attributes.get("type").getValue().toUpperCase());
            } catch (Exception e) {
              System.err.println("[DOT Import] Invalid edge type, defaulting to UNCONSTRAINED");
            }
          }

          // Create a proper EdgeCommand with no command yet
          return EdgeCommand.builder().gamePieceEdge(type).command(null).build();
        });

    // --- Fix 3: actually import the file and verify success ---
    try (Reader r = new FileReader(path)) {
      importer.importGraph(graph, r);
    } catch (Exception ex) {
      throw new RuntimeException("Failed to parse DOT file: " + path, ex);
    }

    // --- Fix 4: assign the edge commands after load ---
    for (EdgeCommand e : graph.edgeSet()) {
      V3_EpsilonSuperstructureStates from = graph.getEdgeSource(e);
      V3_EpsilonSuperstructureStates to = graph.getEdgeTarget(e);
      e.setCommand(getEdgeCommand(from, to, elevator, intake, manipulator));
    }

    System.out.printf(
        "[DOT Import] Loaded %d vertices, %d edges%n",
        graph.vertexSet().size(), graph.edgeSet().size());
  }

  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    @Setter private Command command;
    @Builder.Default private GamePieceEdge gamePieceEdge = GamePieceEdge.UNCONSTRAINED;
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

    if (to.equals(V3_EpsilonSuperstructureStates.BARGE_SCORE)) {
      return Commands.sequence(
          pose.asConfigurationSpaceCommand(elevator, intake, manipulator),
          Commands.waitUntil(() -> manipulator.armInTolerance(Rotation2d.fromDegrees(10))));
    }

    return Commands.sequence(
        pose.asConfigurationSpaceCommand(elevator, intake, manipulator),
        pose.wait(elevator, intake, manipulator));
  }

  /**
   * Adds all predefined edges to the superstructure state graph, categorized by algae condition.
   *
   * @param graph The graph to which edges are added.
   * @param elevator The elevator subsystem.
   * @param manipulator The manipulator subsystem.
   * @param intake The intake subsystem.
   */
  public static void addEdges(
      Graph<V3_EpsilonSuperstructureStates, EdgeCommand> graph,
      ElevatorFSM elevator,
      V3_EpsilonIntake intake,
      V3_EpsilonManipulator manipulator) {
    loadEdgesFromDot(
        Filesystem.getDeployDirectory().toPath().resolve("Superstructure.dot").toString(),
        graph,
        elevator,
        intake,
        manipulator);
  }
}
