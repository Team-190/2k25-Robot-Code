package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.shared.elevator.ElevatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants;
import java.io.FileReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.nio.Attribute;
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

    // Create edges with placeholder commands (null)
    importer.setEdgeWithAttributesFactory(
        attributes -> {
          V3_EpsilonSuperstructureEdges.GamePieceEdge type =
              V3_EpsilonSuperstructureEdges.GamePieceEdge.UNCONSTRAINED;

          if (attributes.containsKey("type")) {
            String value = attributes.get("type").getValue();
            try {
              type = V3_EpsilonSuperstructureEdges.GamePieceEdge.valueOf(value.toUpperCase());
            } catch (IllegalArgumentException e) {
              System.err.println("Unknown edge type: " + value + ", defaulting to UNCONSTRAINED");
            }
          }

          return V3_EpsilonSuperstructureEdges.EdgeCommand.builder()
              .gamePieceEdge(type)
              .command(null) // placeholder
              .build();
        });

    // Map DOT node id -> enum
    importer.setVertexFactory(id -> V3_EpsilonSuperstructureStates.valueOf(id));

    // Capture edge attributes
    Map<EdgeCommand, Map<String, Attribute>> edgeAttrs = new HashMap<>();
    importer.addEdgeWithAttributesConsumer((e, attrs) -> edgeAttrs.put(e, new HashMap<>(attrs)));

    // Import the DOT file
    try (Reader r = new FileReader(path)) {
      importer.importGraph(graph, r);
    } catch (Exception ex) {
      throw new RuntimeException("Failed to parse DOT: " + path, ex);
    }

    // Clear old edge lists
    UNCONSTRAINED.clear();
    NO_ALGAE_EDGES.clear();

    // Iterate over edges to assign commands and categorize
    for (EdgeCommand e : graph.edgeSet()) {
      V3_EpsilonSuperstructureStates from = graph.getEdgeSource(e);
      V3_EpsilonSuperstructureStates to = graph.getEdgeTarget(e);

      Map<String, Attribute> attrs = edgeAttrs.get(e);
      V3_EpsilonSuperstructureEdges.GamePieceEdge type =
          V3_EpsilonSuperstructureEdges.GamePieceEdge.UNCONSTRAINED;

      // START --- ADDED CODE
      // Check for the 'requires' attribute to determine if motion should be
      // sequential
      boolean requiresElevator = false;
      boolean requiresArm = false;
      if (attrs != null && attrs.containsKey("requires")) {
        String value = attrs.get("requires").getValue();
        if ("elevator".equalsIgnoreCase(value)) {
          requiresElevator = true;
        } else if ("arm".equalsIgnoreCase(value)) {
          requiresArm = true;
        }
      }
      e.setRequiresElevator(requiresElevator);
      e.setRequiresArm(requiresArm);
      // END --- ADDED CODE

      if (attrs != null && attrs.get("type") != null) {
        try {
          type =
              V3_EpsilonSuperstructureEdges.GamePieceEdge.valueOf(
                  attrs.get("type").getValue().toUpperCase());
        } catch (IllegalArgumentException ex) {
          System.err.println("Unknown edge type: " + attrs.get("type").getValue());
        }
      }

      // Assign the actual command, passing the new flags
      e.setCommand(
          V3_EpsilonSuperstructureEdges.getEdgeCommand(
              from, to, elevator, intake, manipulator, requiresElevator, requiresArm));

      // Add to proper list
      Edge edge = new Edge(from, to);
      if (type == V3_EpsilonSuperstructureEdges.GamePieceEdge.NO_ALGAE) {
        NO_ALGAE_EDGES.add(edge);
      } else {
        UNCONSTRAINED.add(edge);
      }
    }
  }

  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    @Setter private Command command;
    @Builder.Default private GamePieceEdge gamePieceEdge = GamePieceEdge.UNCONSTRAINED;
    // START --- ADDED CODE
    @Setter @Builder.Default private boolean requiresElevator = false;
    @Setter @Builder.Default private boolean requiresArm = false;
    // END --- ADDED CODE
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
   * @param requiresElevator Flag indicating the elevator must finish its move before other
   *     mechanisms start.
   * @param requiresArm Flag indicating the arm/intake must finish its move before the elevator
   *     starts.
   * @return A {@link Command} that, when executed, transitions the superstructure from the 'from'
   *     state to the 'to' state.
   */
  private static Command getEdgeCommand(
      V3_EpsilonSuperstructureStates from,
      V3_EpsilonSuperstructureStates to,
      ElevatorFSM elevator,
      V3_EpsilonIntake intake,
      V3_EpsilonManipulator manipulator,
      boolean requiresElevator,
      boolean requiresArm) {

    V3_EpsilonSuperstructurePose pose = to.getPose();
    Command moveCommand; // This will hold the command that STARTS the movement.

    if (requiresElevator) {
      // Elevator moves and waits, THEN other mechanisms move.
      moveCommand =
          Commands.sequence(
              pose.setElevatorHeight(elevator),
              elevator.waitUntilAtGoal(),
              pose.asCommand(
                  elevator, intake, manipulator) // CORRECTED: Only move the other subsystems
              );

    } else if (requiresArm) {
      // Arm moves to a safe position and waits, THEN other mechanisms move.
      moveCommand =
          Commands.sequence(
              pose.setManipulatorState(manipulator),
              Commands.waitUntil(manipulator::isSafePosition),
              pose.asCommand(
                  elevator, intake, manipulator) // CORRECTED: Only move the other subsystems
              );
    } else {
      // Default case: All mechanisms move in parallel.
      moveCommand = pose.asCommand(elevator, intake, manipulator);
    }

    if (from == V3_EpsilonSuperstructureStates.HANDOFF) {
      moveCommand =
          Commands.sequence(
              Commands.runOnce(() -> elevator.setPosition(() -> ReefState.ALGAE_MID)),
              elevator.waitUntilAtGoal(),
              moveCommand);
    }
    // move intake out of the way if it will collide
    else if (willCollide(from, to)) {
      moveCommand =
          Commands.sequence(
              Commands.runOnce(() -> intake.setPivotGoal(IntakePivotState.ARM_CLEAR)),
              intake.waitUntilPivotAtGoal(),
              moveCommand);
    }

    if (to == V3_EpsilonSuperstructureStates.HANDOFF) {

      if (requiresArm) {
        moveCommand =
            Commands.sequence(
                pose.setManipulatorState(manipulator),
                Commands.waitUntil(manipulator::isSafePosition),
                Commands.runOnce(() -> elevator.setPosition(() -> ReefState.ALGAE_MID)),
                elevator.waitUntilAtGoal(),
                pose.setIntakeState(intake),
                intake.waitUntilPivotAtGoal(),
                pose.setElevatorHeight(elevator));
      } else if (requiresElevator) {
        moveCommand =
            Commands.sequence(
                Commands.runOnce(() -> elevator.setPosition(() -> ReefState.ALGAE_MID)),
                elevator.waitUntilAtGoal(),
                pose.setManipulatorState(manipulator),
                manipulator.waitUntilArmAtGoal(),
                pose.setIntakeState(intake),
                intake.waitUntilPivotAtGoal(),
                pose.setElevatorHeight(elevator));
      } else {
        moveCommand =
            Commands.sequence(
                Commands.runOnce(() -> elevator.setPosition(() -> ReefState.ALGAE_MID))
                    .alongWith(pose.setManipulatorState(manipulator)),
                Commands.waitSeconds(0.02),
                Commands.waitUntil(() -> elevator.atGoal() || manipulator.armAtGoal()),
                pose.setIntakeState(intake),
                intake.waitUntilPivotAtGoal(),
                pose.setElevatorHeight(elevator));
      }
    }

    // THE CRITICAL FIX:
    // No matter how we start the move, we append a final wait condition.
    // This ensures the command doesn't end until the robot is physically at the
    // target pose.
    return Commands.sequence(moveCommand, waitForPoseCommand(to, elevator, intake, manipulator));
  }

  private static boolean willCollide(
      V3_EpsilonSuperstructureStates from, V3_EpsilonSuperstructureStates to) {

    final int samples = 20; // number of interpolation steps

    for (int i = 0; i <= samples; i++) {
      double t = i / (double) samples;

      // Interpolate elevator height
      double elevHeight =
          ElevatorConstants.ElevatorPositions.getPosition(from.getPose().getElevatorHeight())
                      .getPosition()
                  * (1 - t)
              + ElevatorConstants.ElevatorPositions.getPosition(to.getPose().getElevatorHeight())
                      .getPosition()
                  * t;

      // Interpolate arm angle
      Rotation2d armAngle =
          from.getPose()
              .getArmState()
              .getAngle(Side.POSITIVE)
              .interpolate(to.getPose().getArmState().getAngle(Side.POSITIVE), t);

      // Interpolate intake angle
      Rotation2d intakeAngle =
          from.getPose()
              .getIntakeState()
              .getAngle()
              .interpolate(to.getPose().getIntakeState().getAngle(), t);

      // Arm height
      double armHeight =
          -armAngle.rotateBy(new Rotation2d(-Math.PI / 2)).getSin()
                  * V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS()
              + elevHeight;

      // Arm horizontal extension
      double horiz =
          Math.abs(
              armAngle.rotateBy(new Rotation2d(-Math.PI / 2)).getCos()
                  * V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS());

      // Check safety
      boolean clearsIntake = intakeAngle.getDegrees() > 15 || armHeight > 0.37 || horiz > 0.35;

      if (!clearsIntake) {
        return true; // Collision predicted
      }
    }

    return false; // Safe through transition
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
              // Updated to call new signature with default 'false' values for requirements
              .command(
                  getEdgeCommand(
                      edge.from(), edge.to(), elevator, intake, manipulator, false, false))
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
   * @param intake The intake subsystem.
   */
  public static void addEdges(
      Graph<V3_EpsilonSuperstructureStates, EdgeCommand> graph,
      ElevatorFSM elevator,
      V3_EpsilonIntake intake,
      V3_EpsilonManipulator manipulator) {
    // This method populates the graph by parsing the DOT file.
    loadEdgesFromDot(
        Filesystem.getDeployDirectory().toPath().resolve("Superstructure.dot").toString(),
        graph,
        elevator,
        intake,
        manipulator);

    // NOTE: The two lines below are likely redundant. The loadEdgesFromDot method
    // already
    // adds all edges to the graph. You may want to remove these calls.
    addEdges(graph, UNCONSTRAINED, GamePieceEdge.UNCONSTRAINED, elevator, manipulator, intake);
    addEdges(graph, NO_ALGAE_EDGES, GamePieceEdge.NO_ALGAE, elevator, manipulator, intake);
  }

  // In V3_EpsilonSuperstructureEdges.java

  private static Command waitForPoseCommand(
      V3_EpsilonSuperstructureStates state,
      ElevatorFSM elevator,
      V3_EpsilonIntake intake,
      V3_EpsilonManipulator manipulator) {
    V3_EpsilonSuperstructurePose pose = state.getPose();

    // Add this command to log the check's status
    Command logCheck =
        Commands.runOnce(
            () -> {
              boolean elevatorOk = elevator.atGoal();
              boolean intakeOk = intake.pivotAtGoal();
              boolean armOk = manipulator.armAtGoal();
              System.out.println(
                  "Checking pose for: "
                      + state.toString()
                      + " -> Elevator OK: "
                      + elevatorOk
                      + ", Intake OK: "
                      + intakeOk
                      + ", Arm OK: "
                      + armOk);
            });

    Command wait =
        Commands.sequence(
            Commands.waitSeconds(0.02),
            Commands.waitUntil(
                () -> elevator.atGoal() && intake.pivotAtGoal() && manipulator.armAtGoal()));

    // Run the log once right before starting the wait
    return Commands.sequence(wait, logCheck);
  }
}
