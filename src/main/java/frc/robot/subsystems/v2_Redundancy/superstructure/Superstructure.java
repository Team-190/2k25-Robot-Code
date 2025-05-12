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
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
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
        new SubsystemPoses(
            ReefState.CORAL_INTAKE, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED)),
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
            ReefState.L4_PLUS, ArmState.STOW_DOWN, IntakeState.STOW, FunnelState.OPENED));
    // SCORE_L1(
    //   "L1 CORAL SCORE",

    // ),
    // SCORE_L2,
    // SCORE_L3,
    // SCORE_L4,
    // SCORE_L4_PLUS,
    // INTERMEDIATE_WAIT_FOR_ELEVATOR,
    // INTERMEDIATE_WAIT_FOR_ARM,
    // STOW_UP,
    // FLOOR_ACQUISITION,
    // REEF_ACQUISITION_L2,
    // REEF_ACQUISITION_L3,
    // BARGE,
    // PROCESSOR,
    // INTAKE_FLOOR,
    // INTAKE_REEF_L2,
    // INTAKE_REEF_L3,
    // DROP_REEF_L2,
    // DROP_REEF_L3,
    // SCORE_BARGE,
    // SCORE_PROCESSOR;

    private final String name;
    private SubsystemPoses subsystemPoses;
    private double manipulatorRollerVoltage;
    private double funnelRollerVoltage;
    private double intakeRollerVoltage;

    private SuperstructureStates(String name, SubsystemPoses poses) {
      this.name = name;
      this.subsystemPoses = poses;
    }

    private SuperstructureStates(
        String name,
        double manipulatorRollerVoltage,
        double funnelRollerVoltage,
        double intakeRollerVoltage) {
      this.name = name;
      this.manipulatorRollerVoltage = manipulatorRollerVoltage;
      this.funnelRollerVoltage = funnelRollerVoltage;
      this.intakeRollerVoltage = intakeRollerVoltage;
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
            manipulatorRollerVoltage,
            funnelRollerVoltage,
            intakeRollerVoltage,
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
