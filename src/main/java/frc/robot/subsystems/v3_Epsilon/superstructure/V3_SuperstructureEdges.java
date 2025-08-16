package frc.robot.subsystems.v3_Epsilon.superstructure;

import java.util.ArrayList;
import java.util.List;
import org.jgrapht.graph.DefaultEdge;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulator;
import lombok.Builder;
import lombok.Getter;

public class V3_SuperstructureEdges {

  public static final ArrayList<Edge> AlgaeEdges = new ArrayList<>();
  public static final ArrayList<Edge> CoralEdges = new ArrayList<>();
  public static final ArrayList<Edge> UnconstrainedEdges = new ArrayList<>();
  public static final ArrayList<Edge> NoneEdges = new ArrayList<>();

  public record Edge(V3_SuperstructureStates from, V3_SuperstructureStates to, String action) {
    public Edge(V3_SuperstructureStates from, V3_SuperstructureStates to) {
      this(from, to, "");
    }

    public String toString() {
      return from + " -> " + to + (action.isEmpty() ? "" : " : " + action);
    }
  }

  public enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }

  public enum CoralEdge {
    NONE,
    NO_CORAL,
    CORAL
  }

 @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
    @Builder.Default private final CoralEdge coralEdgeType = CoralEdge.NONE;
  }

  private static Command getEdgeCommand(
      V3_SuperstructureStates from, 
      V3_SuperstructureStates to, 
      Elevator.ElevatorFSM elevator, 
      V3_EpsilonIntake intake, 
      V3_EpsilonManipulator manipulator) {

      // TODO: Implement the actual command logic
        
      return Commands.none();
    }

  public static void createEdges () {
    List<V3_SuperstructureStates> coralPrepLevels = List.of(
        V3_SuperstructureStates.L1_PREP,
        V3_SuperstructureStates.L2_PREP,
        V3_SuperstructureStates.L3_PREP,
        V3_SuperstructureStates.L4_PREP
    );

    List<V3_SuperstructureStates> coralTransitionLevels = List.of(
      V3_SuperstructureStates.L2_TRANSITION,
      V3_SuperstructureStates.L3_TRANSITION,
      V3_SuperstructureStates.L4_TRANSITION
    );

    // Coral Edges
    
    // Level to level edges
    for (V3_SuperstructureStates from : coralPrepLevels) {
      for (V3_SuperstructureStates to : coralPrepLevels) {
        if (from != to) {
          CoralEdges.add(new Edge(from, to, "Coral Level Transition"));
        }
      }
    }

    // Handoff -> transition 
    for (V3_SuperstructureStates to: coralTransitionLevels) {
      CoralEdges.add(new Edge(V3_SuperstructureStates.HANDOFF, to, "Coral Transition"));
    }

    // Handoff -> L1_PREP
    CoralEdges.add(new Edge(V3_SuperstructureStates.L1_PREP, V3_SuperstructureStates.HANDOFF, "Coral Handoff L1 Prep"));

    // Transition -> handoff
    for (V3_SuperstructureStates from: coralTransitionLevels) {
      CoralEdges.add(new Edge(from, V3_SuperstructureStates.HANDOFF, "Coral Handoff"));
    }

    // Coral Prep to Transition
    for (V3_SuperstructureStates from : coralPrepLevels) {
      for (V3_SuperstructureStates to : coralTransitionLevels) {
        CoralEdges.add(new Edge(from, to, "Coral Transition"));
      }
    } 
    
    // Coral Transition to Prep
    CoralEdges.add(new Edge(V3_SuperstructureStates.L2_TRANSITION, V3_SuperstructureStates.L2_PREP, "Coral Transition L2"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.L3_TRANSITION, V3_SuperstructureStates.L3_PREP, "Coral Transition L3"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.L4_TRANSITION, V3_SuperstructureStates.L4_PREP, "Coral Transition L3"));
    

    // Prep to score states
    CoralEdges.add(new Edge(V3_SuperstructureStates.L1_PREP, V3_SuperstructureStates.L1_SCORE, "Coral Score L1"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.L2_PREP, V3_SuperstructureStates.L2_SCORE, "Coral Score L2"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.L3_PREP, V3_SuperstructureStates.L3_SCORE, "Coral Score L3"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.L4_PREP, V3_SuperstructureStates.L4_SCORE, "Coral Score L4"));
    
    // L1_SCORE -> GROUND_INTAKE
    CoralEdges.add(new Edge(V3_SuperstructureStates.L1_PREP, V3_SuperstructureStates.GROUND_INTAKE, "Coral Transition L1"));

    // Algae Edges

    // STOW_DOWN --> L2_ALGAE_PREP
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.STOW_DOWN, V3_SuperstructureStates.L2_ALGAE_PREP, "Algae Prep L2 from STOW DOWN"));

    // Prep --> intake states
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.L2_ALGAE_PREP, V3_SuperstructureStates.L2_ALGAE_INTAKE, "Algae Intake L2"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.L3_ALGAE_PREP, V3_SuperstructureStates.L3_ALGAE_INTAKE, "Algae Intake L3"));

    // Prep <-> Prep States
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.L2_ALGAE_PREP, V3_SuperstructureStates.L3_ALGAE_PREP, "Algae Prep L2 to L3"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.L3_ALGAE_PREP, V3_SuperstructureStates.L2_ALGAE_PREP, "Algae Prep L3 to L2"));

    // BARGE_PREP Edges
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.L2_ALGAE_PREP, V3_SuperstructureStates.BARGE_PREP, "Algae Prep Barge to L2"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.L3_ALGAE_PREP, V3_SuperstructureStates.BARGE_PREP, "Algae Prep Barge to L3"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.STOW_UP, V3_SuperstructureStates.BARGE_PREP, "Algae Prep L2 to Barge"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.BARGE_PREP, V3_SuperstructureStates.BARGE_SCORE, "Algae Prep L2 to Barge"));

    // PROCESSOR Edges
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.PROCESSOR_PREP, V3_SuperstructureStates.PROCESSOR_SCORE, "Algae Processor L2"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.L2_ALGAE_PREP, V3_SuperstructureStates.PROCESSOR_PREP, "Algae Processor L2 Prep"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.L3_ALGAE_PREP, V3_SuperstructureStates.PROCESSOR_PREP, "Algae Processor L3 Prep"));

    // Unconstrained Edges
    
    // BARGE_SCORE -> STOW_DOWN
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.BARGE_SCORE, V3_SuperstructureStates.STOW_DOWN, "BARGE_SCORE to Stow Down"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.BARGE_SCORE, V3_SuperstructureStates.STOW_UP, "BARGE_SCORE to Stow Up"));
    AlgaeEdges.add(new Edge(V3_SuperstructureStates.PROCESSOR_SCORE, V3_SuperstructureStates.STOW_DOWN, "Algae Processor Barge Prep"));

    
  }
}
    
    
    
    
    
    
