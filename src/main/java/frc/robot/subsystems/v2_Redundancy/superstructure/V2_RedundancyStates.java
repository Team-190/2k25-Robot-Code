package frc.robot.subsystems.v2_Redundancy.superstructure;

import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructurePose.SubsystemPoses;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants.FunnelState;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants.IntakeState;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants.ArmState;
import java.util.List;

public class V2_RedundancyStates {
  public static final List<SuperstructureStates> Actions =
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
        new SubsystemPoses(
            ReefState.L1, ArmState.STOW_DOWN, IntakeState.L1_EXT, FunnelState.OPENED)),
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

    public V2_RedundancySuperstructureState createState(
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
}
