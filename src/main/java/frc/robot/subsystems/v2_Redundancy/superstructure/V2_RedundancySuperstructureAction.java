package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import lombok.Getter;

public class V2_RedundancySuperstructureAction extends SuperstructureState {
  @Getter private final double manipulatorRollerVoltage;
  @Getter private final double funnelRollerVoltage;
  @Getter private final double intakeRollerVoltage;

  public V2_RedundancySuperstructureAction(
      String key,
      double manipulatorRollerVoltage,
      double funnelRollerVoltage,
      double intakeRollerVoltage,
      V2_RedundancyElevator elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
    super(key, elevator, manipulator, funnel, intake);
    this.manipulatorRollerVoltage = manipulatorRollerVoltage;
    this.funnelRollerVoltage = funnelRollerVoltage;
    this.intakeRollerVoltage = intakeRollerVoltage;
  }

  public Command runManipulator() {
    return super.manipulator.runManipulator(manipulatorRollerVoltage);
  }

  public Command runFunnel() {
    return super.funnel.setFunnelVoltage(funnelRollerVoltage);
  }

  public Command runIntake() {
    return super.intake.setRollerVoltage(intakeRollerVoltage);
  }

  public Command action() {
    return Commands.parallel(runManipulator(), runFunnel(), runIntake());
  }
}
