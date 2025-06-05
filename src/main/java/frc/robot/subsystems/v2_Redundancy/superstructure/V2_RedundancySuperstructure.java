package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import frc.robot.util.NTPrefixes;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Queue;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancySuperstructure extends SubsystemBase {

  private final Graph<SuperstructureStates, EdgeCommand> graph;
  @Getter private final V2_RedundancyElevator elevator;
  @Getter private final V2_RedundancyFunnel funnel;
  @Getter private final V2_RedundancyManipulator manipulator;
  @Getter private final V2_RedundancyIntake intake;

  private SuperstructureStates currentState;
  private SuperstructureStates nextState;

  private SuperstructureStates targetState;
  private EdgeCommand edgeCommand;

  public enum SuperstructureStates {
    START("START", new SubsystemPoses()),
    STOW_DOWN(
        "STOW DOWN",
        new SubsystemPoses(
            ReefState.STOW, ArmState.STOW_DOWN,
            IntakeState.STOW, FunnelState.OPENED)),
    INTAKE(
        "INTAKE CORAL",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.CORAL_INTAKE_VOLTS().get(),
            12.0,
            0.0)),
    L1(
        "L1 CORAL SETPOINT",
        new SubsystemPoses(ReefState.L1, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    L2(
        "L2 CORAL SETPOINT",
        new SubsystemPoses(ReefState.L2, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    L3(
        "L3 CORAL SETPOINT",
        new SubsystemPoses(ReefState.L3, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    L4(
        "L4 CORAL SETPOINT",
        new SubsystemPoses(ReefState.L4, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    L4_PLUS(
        "L4+ CORAL SETPOINT",
        new SubsystemPoses(
            ReefState.L4_PLUS, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
    SCORE_L1(
        "L1 CORAL SCORE",
        List.of(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.L1_VOLTS().get(), 0.0, 0.0)),
    SCORE_L2(
        "L2 CORAL SCORE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_CORAL_VOLTS().get(), 0.0, 0.0)),
    SCORE_L3(
        "L3 CORAL SCORE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_CORAL_VOLTS().get(), 0.0, 0.0)),
    SCORE_L4(
        "L4 CORAL SCORE",
        List.of(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.L4_VOLTS().get(), 0.0, 0.0)),
    SCORE_L4_PLUS(
        "L4+ CORAL SCORE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_CORAL_VOLTS().get(), 0.0, 0.0)),
    INTERMEDIATE_WAIT_FOR_ELEVATOR(
        "WAIT FOR ELEVATOR",
        new SubsystemPoses(
            ReefState.ALGAE_MID,
            ArmState.STOW_DOWN,
            IntakeState.STOW,
            FunnelState.OPENED)), // TODO: Check this
    INTERMEDIATE_WAIT_FOR_ARM(
        "WAIT FOR ARM",
        new SubsystemPoses(
            ReefState.ALGAE_MID,
            ArmState.STOW_DOWN,
            IntakeState.STOW,
            FunnelState.OPENED)), // TODO: Check this
    STOW_UP(
        "STOW UP",
        new SubsystemPoses(ReefState.STOW, ArmState.STOW_UP, IntakeState.STOW, FunnelState.OPENED)),
    FLOOR_ACQUISITION(
        "FLOOR ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_FLOOR_INTAKE,
            ArmState.FLOOR_INTAKE,
            IntakeState.INTAKE,
            FunnelState.OPENED)),
    REEF_ACQUISITION_L2(
        "L2 ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_BOTTOM,
            ArmState.REEF_INTAKE,
            IntakeState.STOW,
            FunnelState.OPENED)),
    REEF_ACQUISITION_L3(
        "L3 ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_INTAKE_TOP,
            ArmState.REEF_INTAKE,
            IntakeState.STOW,
            FunnelState.OPENED)),
    BARGE(
        "BARGE SETPOINT",
        new SubsystemPoses(
            ReefState.ALGAE_SCORE, ArmState.STOW_UP, IntakeState.STOW, FunnelState.CLOSED)),
    PROCESSOR(
        "PROCESSOR SETPOINT",
        new SubsystemPoses(
            ReefState.STOW, ArmState.PROCESSOR, IntakeState.STOW, FunnelState.CLOSED)),
    INTAKE_FLOOR(
        "INTAKE FLOOR",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get(),
            0.0,
            6.0)),
    INTAKE_REEF_L2(
        "L2 ALGAE INTAKE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get(),
            0.0,
            0.0)),
    INTAKE_REEF_L3(
        "L3 ALGAE INTAKE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().get(),
            0.0,
            0.0)),
    DROP_REEF_L2(
        "DROP L2 ALGAE",
        List.of(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.REMOVE_ALGAE().get(), 0.0, 0.0)),
    DROP_REEF_L3(
        "DROP L3 ALGAE",
        List.of(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.REMOVE_ALGAE().get(), 0.0, 0.0)),
    SCORE_BARGE(
        "SCORE BARGE",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_ALGAE_VOLTS().get(), 0.0, 0.0)),
    SCORE_PROCESSOR(
        "SCORE PROCESSOR",
        List.of(
            V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_ALGAE_VOLTS().get(), 0.0, 0.0)),
    CLIMB(
        "CLIMB",
        new SubsystemPoses(
            ReefState.STOW, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.CLIMB)),
    FUNNEL_CLOSE_WITH_STOW_UP(
        "FUNNEL CLOSE WITH STOW UP",
        new SubsystemPoses(
            ReefState.STOW,
            ArmState.STOW_UP,
            IntakeState.STOW,
            FunnelState.CLOSED)), // TODO: Check this and add edges
    FUNNEL_CLOSE_WITH_STOW_DOWN(
        "FUNNEL CLOSE WITH STOW DOWN",
        new SubsystemPoses(
            ReefState.STOW,
            ArmState.STOW_DOWN,
            IntakeState.STOW,
            FunnelState.CLOSED)); // TODO: Check this and add edges
    private final String name;
    private SubsystemPoses subsystemPoses;
    private List<Double> voltages;

    private SuperstructureStates(String name, SubsystemPoses poses) {
      this.name = name;
      this.subsystemPoses = poses;
      this.voltages = null;
    }

    private SuperstructureStates(String name, List<Double> voltages) {
      this.name = name;
      this.subsystemPoses = null;
      this.voltages = voltages;
    }

    public SuperstructureState createState(
        V2_RedundancyElevator elevator,
        V2_RedundancyFunnel funnel,
        V2_RedundancyManipulator manipulator,
        V2_RedundancyIntake intake) {
      if (subsystemPoses != null) {
        return new V2_RedundancySuperstructurePose(
            name, subsystemPoses, elevator, funnel, manipulator, intake);
      } else {
        return new V2_RedundancySuperstructureAction(
            name,
            voltages.get(0),
            voltages.get(1),
            voltages.get(2),
            elevator,
            manipulator,
            funnel,
            intake);
      }
    }

    @Override
    public String toString() {
      return name;
    }
  }

  private static final List<SuperstructureStates> actions =
      List.of(
          SuperstructureStates.INTAKE,
          SuperstructureStates.SCORE_L1,
          SuperstructureStates.SCORE_L2,
          SuperstructureStates.SCORE_L3,
          SuperstructureStates.SCORE_L4,
          SuperstructureStates.SCORE_L4_PLUS,
          SuperstructureStates.INTAKE_FLOOR,
          SuperstructureStates.INTAKE_REEF_L2,
          SuperstructureStates.INTAKE_REEF_L3,
          SuperstructureStates.DROP_REEF_L2,
          SuperstructureStates.DROP_REEF_L3,
          SuperstructureStates.SCORE_BARGE,
          SuperstructureStates.SCORE_PROCESSOR);

  public V2_RedundancySuperstructure(
      V2_RedundancyElevator elevator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake) {
    this.elevator = elevator;
    this.funnel = funnel;
    this.manipulator = manipulator;
    this.intake = intake;

    currentState = SuperstructureStates.START;
    nextState = null;

    targetState = SuperstructureStates.START;

    // Initialize the graph
    graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    for (SuperstructureStates vertex : SuperstructureStates.values()) {
      graph.addVertex(vertex);
    }

    // Add edges between states
    addEdges();
  }

  public SuperstructureStates getCurrentState() {
    return currentState;
  }

  public SuperstructureStates getNextState() {
    return nextState;
  }

  public SuperstructureStates getTargetState() {
    return targetState;
  }

  private void addEdges() {

    // CORAL-RELATED STATES
    List<SuperstructureStates> coralLevels =
        List.of(
            SuperstructureStates.L1,
            SuperstructureStates.L2,
            SuperstructureStates.L3,
            SuperstructureStates.L4);

    // stow_down <-> each coral level (bidirectional, no algae)
    for (SuperstructureStates level : coralLevels) {
      addEdge(SuperstructureStates.STOW_DOWN, level, true, AlgaeEdge.NO_ALGAE, false);
    }

    // every coral level <-> every other coral level (no algae)
    for (SuperstructureStates from : coralLevels) {
      for (SuperstructureStates to : coralLevels) {
        if (from != to) {
          addEdge(from, to, AlgaeEdge.NONE);
        }
      }
    }

    // Misellaneous L4+ transitions
    addEdge(SuperstructureStates.L4_PLUS, SuperstructureStates.L4, true, AlgaeEdge.NONE, false);
    addEdge(SuperstructureStates.L4_PLUS, SuperstructureStates.STOW_DOWN, AlgaeEdge.NO_ALGAE);
    addEdge(
        SuperstructureStates.L4_PLUS,
        SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
        AlgaeEdge.NO_ALGAE);

    // each “level” → its scoring state (one‑way, no algae)
    Map<SuperstructureStates, SuperstructureStates> coralScoreMap =
        Map.of(
            SuperstructureStates.L1, SuperstructureStates.SCORE_L1,
            SuperstructureStates.L2, SuperstructureStates.SCORE_L2,
            SuperstructureStates.L3, SuperstructureStates.SCORE_L3,
            SuperstructureStates.L4, SuperstructureStates.SCORE_L4,
            SuperstructureStates.L4_PLUS, SuperstructureStates.SCORE_L4_PLUS);
    coralScoreMap.forEach((level, score) -> addEdge(level, score, true, AlgaeEdge.NO_ALGAE, false));

    // each coral level → FLOOR_ACQUISITION and → INTERMEDIATE_WAIT_FOR_ELEVATOR (one‑way)
    for (SuperstructureStates level : List.of(SuperstructureStates.L1, SuperstructureStates.L2)) {
      for (SuperstructureStates target :
          List.of(
              SuperstructureStates.FLOOR_ACQUISITION,
              SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR)) {
        addEdge(level, target, AlgaeEdge.NONE);
      }
    }

    for (SuperstructureStates level : List.of(SuperstructureStates.L3, SuperstructureStates.L4)) {
      for (SuperstructureStates target :
          List.of(
              SuperstructureStates.REEF_ACQUISITION_L2,
              SuperstructureStates.REEF_ACQUISITION_L3,
              SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM)) {
        addEdge(level, target, AlgaeEdge.NONE);
      }
    }

    // INTAKE_CORAL ↔ STOW_DOWN (bidirectional, no algae)
    addEdge(
        SuperstructureStates.INTAKE,
        SuperstructureStates.STOW_DOWN,
        true,
        AlgaeEdge.NO_ALGAE,
        false);

    // INTERMEDIATE_WAIT_FOR_ELEVATOR TRANSITIONS (all one‑way, no algae)
    List<SuperstructureStates> iveDestinations =
        List.of(
            SuperstructureStates.STOW_UP,
            SuperstructureStates.REEF_ACQUISITION_L2,
            SuperstructureStates.REEF_ACQUISITION_L3,
            SuperstructureStates.BARGE,
            SuperstructureStates.PROCESSOR);
    for (SuperstructureStates dest : iveDestinations) {
      addEdge(SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR, dest, AlgaeEdge.ALGAE);
    }

    // INTERMEDIATE_WAIT_FOR_ARM TRANSITIONS (one‑way, no algae)
    List<SuperstructureStates> iwaDestinations =
        List.of(
            SuperstructureStates.STOW_DOWN,
            SuperstructureStates.L1,
            SuperstructureStates.L2,
            SuperstructureStates.FLOOR_ACQUISITION);
    for (SuperstructureStates dest : iwaDestinations) {
      addEdge(SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM, dest, AlgaeEdge.NO_ALGAE);
    }

    // STOW_UP → multiple targets (one‑way, no algae)
    List<SuperstructureStates> stowUpDestinations =
        List.of(
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            SuperstructureStates.BARGE,
            SuperstructureStates.PROCESSOR);
    for (SuperstructureStates dest : stowUpDestinations) {
      addEdge(SuperstructureStates.STOW_UP, dest, AlgaeEdge.ALGAE);
    }

    // FLOOR_ACQUISITION → multiple targets (one‑way, no algae)
    List<SuperstructureStates> floorAcqDest =
        List.of(
            SuperstructureStates.STOW_DOWN,
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR,
            SuperstructureStates.INTAKE_FLOOR);
    for (SuperstructureStates dest : floorAcqDest) {
      addEdge(SuperstructureStates.FLOOR_ACQUISITION, dest, AlgaeEdge.NO_ALGAE);
    }

    // REEF-RELATED ACQUISITION STATES (using algaeMap style)
    // Define “from → to” for algae acquisition (one‑way with ALGAE), then add reverse with
    // NO_ALGAE
    Map<SuperstructureStates, List<SuperstructureStates>>
        reefMap = // Algae states here are probably wrong
        Map.of(
                SuperstructureStates.REEF_ACQUISITION_L2,
                    List.of(
                        SuperstructureStates.INTAKE_REEF_L2,
                        SuperstructureStates.DROP_REEF_L2,
                        SuperstructureStates.REEF_ACQUISITION_L3,
                        SuperstructureStates.BARGE,
                        SuperstructureStates.PROCESSOR,
                        SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
                        SuperstructureStates.STOW_UP),
                SuperstructureStates.REEF_ACQUISITION_L3,
                    List.of(
                        SuperstructureStates.INTAKE_REEF_L3,
                        SuperstructureStates.DROP_REEF_L3,
                        SuperstructureStates.REEF_ACQUISITION_L2,
                        SuperstructureStates.BARGE,
                        SuperstructureStates.PROCESSOR,
                        SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
                        SuperstructureStates.STOW_UP));
    reefMap.forEach(
        (from, targets) -> {
          for (SuperstructureStates to : targets) {
            // forward edge uses ALGAE
            addEdge(from, to, false, AlgaeEdge.ALGAE, false);
            // reverse edge uses NO_ALGAE
            addEdge(to, from, false, AlgaeEdge.NO_ALGAE, false);
          }
        });

    // BARGE and PROCESSOR transitions (one‑way, no algae)
    for (SuperstructureStates dest :
        List.of(SuperstructureStates.PROCESSOR, SuperstructureStates.SCORE_BARGE)) {
      addEdge(SuperstructureStates.BARGE, dest, AlgaeEdge.ALGAE);
    }
    for (SuperstructureStates dest :
        List.of(
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            SuperstructureStates.REEF_ACQUISITION_L2,
            SuperstructureStates.REEF_ACQUISITION_L3)) {
      addEdge(SuperstructureStates.BARGE, dest, AlgaeEdge.NO_ALGAE);
    }

    addEdge(SuperstructureStates.SCORE_BARGE, SuperstructureStates.BARGE, AlgaeEdge.NONE);

    for (SuperstructureStates dest :
        List.of(SuperstructureStates.BARGE, SuperstructureStates.SCORE_PROCESSOR)) {
      addEdge(SuperstructureStates.PROCESSOR, dest, AlgaeEdge.ALGAE);
    }
    for (SuperstructureStates dest :
        List.of(
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM,
            SuperstructureStates.REEF_ACQUISITION_L2,
            SuperstructureStates.REEF_ACQUISITION_L3)) {
      addEdge(SuperstructureStates.PROCESSOR, dest, AlgaeEdge.NO_ALGAE);
    }
    addEdge(SuperstructureStates.SCORE_PROCESSOR, SuperstructureStates.PROCESSOR, AlgaeEdge.NONE);

    // FLOOR_INTAKE, REEF_INTAKE, REEF_DROP transitions (one‑way, no algae)
    addEdge(
        SuperstructureStates.INTAKE_FLOOR, SuperstructureStates.FLOOR_ACQUISITION, AlgaeEdge.NONE);
    addEdge(
        SuperstructureStates.INTAKE_REEF_L2,
        SuperstructureStates.REEF_ACQUISITION_L2,
        AlgaeEdge.NONE);
    addEdge(
        SuperstructureStates.INTAKE_REEF_L3,
        SuperstructureStates.REEF_ACQUISITION_L3,
        AlgaeEdge.NONE);
    addEdge(
        SuperstructureStates.DROP_REEF_L2,
        SuperstructureStates.REEF_ACQUISITION_L2,
        AlgaeEdge.NONE);
    addEdge(
        SuperstructureStates.DROP_REEF_L3,
        SuperstructureStates.REEF_ACQUISITION_L3,
        AlgaeEdge.NONE);

    // START → STOW_DOWN (one‑way, no algae)
    addEdge(SuperstructureStates.START, SuperstructureStates.STOW_DOWN, AlgaeEdge.NONE);

    // Stow Down transitions
    for (SuperstructureStates dest :
        List.of(
            SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR,
            SuperstructureStates.FLOOR_ACQUISITION)) {
      addEdge(SuperstructureStates.STOW_DOWN, dest, AlgaeEdge.NO_ALGAE);
    }

    // STOW_DOWN <-> CLIMB
    addEdge(
        SuperstructureStates.STOW_DOWN,
        SuperstructureStates.CLIMB,
        true,
        AlgaeEdge.NO_ALGAE,
        false);

    addEdge(
        SuperstructureStates.STOW_DOWN,
        SuperstructureStates.FUNNEL_CLOSE_WITH_STOW_DOWN,
        true,
        AlgaeEdge.NO_ALGAE,
        false);
    addEdge(
        SuperstructureStates.STOW_UP,
        SuperstructureStates.FUNNEL_CLOSE_WITH_STOW_UP,
        true,
        AlgaeEdge.ALGAE,
        false);
  }

  @Override
  public void periodic() {
    if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
      // Update edge to new state
      if (nextState != null) {
        currentState = nextState;
        nextState = null;
      }

      // Schedule next command in sequence
      if (currentState != targetState) {
        bfs(currentState, targetState)
            .ifPresent(
                next -> {
                  this.nextState = next;
                  edgeCommand = graph.getEdge(currentState, next);
                  edgeCommand.getCommand().schedule();
                });
      }
    }
    Logger.recordOutput(NTPrefixes.SUPERSTRUCTURE + "Goal", targetState);
    Logger.recordOutput(NTPrefixes.SUPERSTRUCTURE + "Current State", currentState);
    Logger.recordOutput(NTPrefixes.SUPERSTRUCTURE + "Next State", nextState);
  }

  private void addEdge(SuperstructureStates from, SuperstructureStates to, AlgaeEdge algaeEdge) {
    addEdge(from, to, false, algaeEdge, false);
  }

  private void addEdge(
      SuperstructureStates from,
      SuperstructureStates to,
      boolean reverse,
      AlgaeEdge algaeEdge,
      boolean restricted) {
    graph.addEdge(
        from,
        to,
        EdgeCommand.builder()
            .command(getEdgeCommand(from, to))
            .algaeEdgeType(algaeEdge)
            .restricted(restricted)
            .build());
    if (reverse) {
      graph.addEdge(
          to,
          from,
          EdgeCommand.builder()
              .command(getEdgeCommand(to, from))
              .algaeEdgeType(algaeEdge)
              .restricted(restricted)
              .build());
    }
  }

  public Command runGoal(SuperstructureStates goal) {
    return runOnce(() -> setGoal(goal)).andThen(Commands.idle(this));
  }

  public Command runGoal(Supplier<SuperstructureStates> goal) {
    return run(() -> setGoal(goal.get()));
  }

  private Command getEdgeCommand(SuperstructureStates from, SuperstructureStates to) {
    if (actions.contains(to)) {
      return to.createState(elevator, funnel, manipulator, intake).action();
    }
    V2_RedundancySuperstructurePose pose =
        (V2_RedundancySuperstructurePose) to.createState(elevator, funnel, manipulator, intake);

    if (from == SuperstructureStates.INTAKE_FLOOR) {
      return Commands.parallel(
          pose.action(), intake.setRollerVoltage(-6).withTimeout(1)); // TODO: Check this
    }

    if (to == SuperstructureStates.INTERMEDIATE_WAIT_FOR_ARM
        || (from == SuperstructureStates.FLOOR_ACQUISITION && to == SuperstructureStates.STOW_DOWN)
        || to == SuperstructureStates.STOW_UP) {
      return pose.setArmState()
          .andThen(
              pose.setIntakeState()
                  .alongWith(pose.setElevatorHeight())
                  .alongWith(pose.setFunnelState()));
    }
    if (to == SuperstructureStates.INTERMEDIATE_WAIT_FOR_ELEVATOR) {
      return pose.setElevatorHeight()
          .andThen(
              pose.setIntakeState().alongWith(pose.setArmState()).alongWith(pose.setFunnelState()));
    }

    return pose.action(); // need to determine order based on from and to
  }

  private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureStates goal) {
    Logger.recordOutput(
        NTPrefixes.SUPERSTRUCTURE + "EdgeAllowed/" + goal + "/" + edge,
        (!edge.isRestricted() || goal == graph.getEdgeTarget(edge))
            && (edge.getAlgaeEdgeType() == AlgaeEdge.NONE
                || RobotState.isHasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE)));
    return (!edge.isRestricted() || goal == graph.getEdgeTarget(edge))
        && (edge.getAlgaeEdgeType() == AlgaeEdge.NONE
            || RobotState.isHasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE));
  }

  public void setAutoStart() {
    currentState = SuperstructureStates.START;
    nextState = null;
    targetState = SuperstructureStates.STOW_DOWN;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
  }

  private Optional<SuperstructureStates> bfs(
      SuperstructureStates start, SuperstructureStates goal) {
    // Map to track the parent of each visited node
    Map<SuperstructureStates, SuperstructureStates> parents = new HashMap<>();
    Queue<SuperstructureStates> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      SuperstructureStates current = queue.poll();
      // Check if we've reached the goal
      if (current == goal) {
        break;
      }
      // Process valid neighbors
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        SuperstructureStates neighbor = graph.getEdgeTarget(edge);
        // Only process unvisited neighbors
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    // Reconstruct the path to the goal if found
    if (!parents.containsKey(goal)) {
      return Optional.empty(); // Goal not reachable
    }

    // Trace back the path from goal to start
    SuperstructureStates nextState = goal;
    while (!nextState.equals(start)) {
      SuperstructureStates parent = parents.get(nextState);
      if (parent == null) {
        return Optional.empty(); // No valid path found
      } else if (parent.equals(start)) {
        // Return the edge from start to the next node
        return Optional.of(nextState);
      }
      nextState = parent;
    }
    return Optional.of(nextState);
  }

  private void setGoal(SuperstructureStates goal) {
    // Don't do anything if goal is the same
    if (this.targetState == goal) return;
    this.targetState = goal;

    if (nextState == null) return;

    var edgeToCurrentState = graph.getEdge(nextState, currentState);
    // Figure out if we should schedule a different command to get to goal faster
    if (edgeCommand.getCommand().isScheduled()
        && edgeToCurrentState != null
        && isEdgeAllowed(edgeToCurrentState, goal)) {
      // Figure out where we would have gone from the previous state
      bfs(currentState, goal)
          .ifPresent(
              newNext -> {
                if (newNext == nextState) {
                  // We are already on track
                  return;
                }

                if (newNext != currentState && graph.getEdge(nextState, newNext) != null) {
                  // We can skip directly to the newNext edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(currentState, newNext);
                  edgeCommand.getCommand().schedule();
                  nextState = newNext;
                } else {
                  // Follow the reverse edge from next back to the current edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(nextState, currentState);
                  edgeCommand.getCommand().schedule();
                  var temp = currentState;
                  currentState = nextState;
                  nextState = temp;
                }
              });
    }
  }

  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final boolean restricted = false;
    @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  }

  private enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }

  public Command setPosition() {
    switch (RobotState.getOIData().currentReefHeight()) {
      case STOW, CORAL_INTAKE -> {
        return runGoal(SuperstructureStates.STOW_DOWN);
      }
      case L1 -> {
        return runGoal(SuperstructureStates.L1);
      }
      case L2 -> {
        return runGoal(SuperstructureStates.L2);
      }
      case L3 -> {
        return runGoal(SuperstructureStates.L3);
      }
      case L4 -> {
        return runGoal(SuperstructureStates.L4);
      }
      default -> {
        return Commands.none();
      }
    }
  }
}
