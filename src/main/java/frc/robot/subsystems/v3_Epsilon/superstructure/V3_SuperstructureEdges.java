package frc.robot.subsystems.v3_Epsilon.superstructure;

import java.util.ArrayList;

public class V3_SuperstructureEdges {

  public static final ArrayList<Edge> AlgaeEdges = new ArrayList<>();
  public static final ArrayList<Edge> CoralEdges = new ArrayList<>();

  public record Edge(V3_SuperstructureStates from, V3_SuperstructureStates to, String action) {
    public Edge(V3_SuperstructureStates from, V3_SuperstructureStates to) {
      this(from, to, "");
    }

    public String toString() {
      return String.format("Edge[from=%s, to=%s, action=%s]", from.name(), to.name(), action);
    }
  }

  public enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }

  // @Builder(toBuilder = true)
  // @Getter
  // public static class EdgeCommand extends DefaultEdge {
  //   private final Command command;
  //   @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  // }

  // private static Command getEdgeCommand(
  //     V3_SuperstructureStates from, V3_SuperstructureStates to, AlgaeEdge algaeEdgeType) {
  //   return Command.builder().from(from).to(to).algaeEdgeType(algaeEdgeType).build();
  // }
}
