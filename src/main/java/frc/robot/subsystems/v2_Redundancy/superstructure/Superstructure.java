package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v2_Redundancy.superstructure.SuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import java.util.List;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

public class Superstructure {

  private final Graph<SuperstructureState, EdgeCommand> graph;
  private final V2_RedundancyElevator elevator;
  private final V2_RedundancyFunnel funnel;
  private final V2_RedundancyManipulator manipulator;
  private final V2_RedundancyIntake intake;

  private SuperstructureState previousState;
  private SuperstructureState currentState;
  private SuperstructureState nextState;

  private SuperstructureState targetState;

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
            0.0
        )),
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
            FunnelState.OPENED)), // TODO: Fix this
    // INTERMEDIATE_WAIT_FOR_ARM,
    STOW_UP(
        "STOW UP",
        new SubsystemPoses(ReefState.STOW, ArmState.STOW_UP, IntakeState.STOW, FunnelState.OPENED)),
    FLOOR_ACQUISITION(
        "FLOOR ALGAE SETPOINT",
        new SubsystemPoses(
            ReefState.STOW, ArmState.FLOOR_INTAKE, IntakeState.INTAKE, FunnelState.OPENED)),
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
    ;
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

    public SuperstructureState createPose(
        V2_RedundancyElevator elevator,
        V2_RedundancyFunnel funnel,
        V2_RedundancyManipulator manipulator,
        V2_RedundancyIntake intake) {
      if (subsystemPoses != null) {
        return new SuperstructurePose(name, subsystemPoses, elevator, funnel, manipulator, intake);
      } else {
        return new SuperstructureAction(
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

  private Superstructure(
      V2_RedundancyElevator elevator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake) {
    this.elevator = elevator;
    this.funnel = funnel;
    this.manipulator = manipulator;
    this.intake = intake;

    previousState = SuperstructureStates.START.createPose(elevator, funnel, manipulator, intake);
    currentState = SuperstructureStates.START.createPose(elevator, funnel, manipulator, intake);
    nextState = SuperstructureStates.START.createPose(elevator, funnel, manipulator, intake);

    targetState = SuperstructureStates.START.createPose(elevator, funnel, manipulator, intake);

    graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    for (SuperstructureStates state : SuperstructureStates.values()) {
      SuperstructureState vertex = state.createPose(elevator, funnel, manipulator, intake);
      graph.addVertex(vertex);
    }
  }

  public class EdgeCommand extends DefaultEdge {

    private final Command command;

    public EdgeCommand(Command command) {
      this.command = command;
    }

    public Command getCommand() {
      return command;
    }
  }
}
