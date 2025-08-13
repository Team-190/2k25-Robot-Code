package frc.robot.subsystems.v3_Epsilon.superstructure;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

import org.jgrapht.graph.DefaultEdge;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;

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

  public enum GamePieceEdge {
    NONE, // No game pieces
    ALGAE, // Algae only
    CORAL, // Coral only
    ALGAE_CORAL // Can have either algae or coral
  }

 @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final GamePieceEdge edgeType = GamePieceEdge.NONE;
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
    List<V3_SuperstructureStates> coralPrepLevels = List.of( // Except L1_PREP, which is handled separately
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
    UnconstrainedEdges.add(new Edge(V3_SuperstructureStates.L1_SCORE, V3_SuperstructureStates.HANDOFF, "Coral Handoff L1 Score"));

    // Transition -> handoff
    for (V3_SuperstructureStates from: coralTransitionLevels) {
      CoralEdges.add(new Edge(from, V3_SuperstructureStates.HANDOFF, "Coral Handoff"));
    }

    // Coral Transition to Prep and reverse
    for (V3_SuperstructureStates from : coralTransitionLevels) {
      for (V3_SuperstructureStates to : coralPrepLevels) {
        CoralEdges.add(new Edge(from, to, "Coral Transition"));
        CoralEdges.add(new Edge(to, from, "Coral Transition Reverse"));
      }
    }

    // Prep to other transition states
    for (V3_SuperstructureStates from : coralPrepLevels) {
      for (V3_SuperstructureStates to : coralTransitionLevels) {
        if ((from == V3_SuperstructureStates.L2_PREP && to == V3_SuperstructureStates.L2_TRANSITION) ||
            (from == V3_SuperstructureStates.L3_PREP && to == V3_SuperstructureStates.L3_TRANSITION) ||
            (from == V3_SuperstructureStates.L4_PREP && to == V3_SuperstructureStates.L4_TRANSITION)) {
          continue; 
        }
        CoralEdges.add(new Edge(from, to, "Coral Prep Transition"));
      }
    }

    // Prep to score states
    CoralEdges.add(new Edge(V3_SuperstructureStates.L1_PREP, V3_SuperstructureStates.L1_SCORE, "Coral Score L1"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.L2_PREP, V3_SuperstructureStates.L2_SCORE, "Coral Score L2"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.L3_PREP, V3_SuperstructureStates.L3_SCORE, "Coral Score L3"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.L4_PREP, V3_SuperstructureStates.L4_SCORE, "Coral Score L4"));
    
    // L1_SCORE <-> GROUND_INTAKE
    CoralEdges.add(new Edge(V3_SuperstructureStates.L1_PREP, V3_SuperstructureStates.GROUND_INTAKE, "Coral Transition L1"));
    NoneEdges.add(new Edge(V3_SuperstructureStates.L1_SCORE, V3_SuperstructureStates.GROUND_INTAKE, "Coral Transition L1 Score"));
    CoralEdges.add(new Edge(V3_SuperstructureStates.GROUND_INTAKE, V3_SuperstructureStates.L1_PREP, "Coral Transition L1 Ground Intake"));

    // IWFE states
    for (V3_SuperstructureStates from : Stream.concat(coralPrepLevels.stream(), Stream.of(V3_SuperstructureStates.L2_SCORE,V3_SuperstructureStates.L3_SCORE,V3_SuperstructureStates.L4_SCORE)).toList()) {
      UnconstrainedEdges.add(new Edge(from, V3_SuperstructureStates.INTERMIDIATE_WAIT_FOR_ELEVATOR, "Coral IWFE Transition"));
    }
    UnconstrainedEdges.add(new Edge(V3_SuperstructureStates.STOW_DOWN, V3_SuperstructureStates.INTERMIDIATE_WAIT_FOR_ELEVATOR, "Coral IWFE Transition Stow Down"));
    UnconstrainedEdges.add(new Edge(V3_SuperstructureStates.INTERMIDIATE_WAIT_FOR_ELEVATOR, V3_SuperstructureStates.HANDOFF, "Coral IWFE Transition Handoff"));

    // Stow Down to Ground Intake and Stow Up
    NoneEdges.add(new Edge(V3_SuperstructureStates.STOW_DOWN, V3_SuperstructureStates.GROUND_INTAKE, "Coral Stow Down Ground Intake"));
    NoneEdges.add(new Edge(V3_SuperstructureStates.STOW_DOWN, V3_SuperstructureStates.STOW_UP, "Coral Stow Down Stow Up")); //Optional coral


    // Algae Edges
    List<V3_SuperstructureStates> algaePrepStates = List.of(
        V3_SuperstructureStates.L2_ALGAE_PREP,
        V3_SuperstructureStates.L3_ALGAE_PREP,
        V3_SuperstructureStates.BARGE_PREP,
        V3_SuperstructureStates.PROCESSOR_PREP
    );
    List<V3_SuperstructureStates> algaeScoreStates = List.of(
        V3_SuperstructureStates.L2_ALGAE_SCORE,
        V3_SuperstructureStates.L3_ALGAE_SCORE,
        V3_SuperstructureStates.BARGE_SCORE,
        V3_SuperstructureStates.PROCESSOR_SCORE
    );

    for (int i = 0; i < 4; i++) {
      NoneEdges.add(new Edge(algaePrepStates.get(i), algaeScoreStates.get(i), "Algae Score Transition"));
      NoneEdges.add(new Edge(algaeScoreStates.get(i), algaePrepStates.get(i), "Algae Prep Transition"));
    }

    for (V3_SuperstructureStates to : algaePrepStates) {
      NoneEdges.add(new Edge(V3_SuperstructureStates.STOW_UP, to, "Algae Stow Up Transition"));
    }

    

  }
}
    
    
    
    
    
    
