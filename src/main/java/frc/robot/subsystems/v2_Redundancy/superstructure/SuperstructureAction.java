package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.Elevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.Funnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.Intake;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.Manipulator;
import lombok.Getter;

public class SuperstructureAction extends SuperstructureState {
  @Getter private final double manipulatorRollerVoltage;
  @Getter private final double funnelRollerVoltage;
  @Getter private final double intakeRollerVoltage;

  public SuperstructureAction(
      String key,
      double manipulatorRollerVoltage,
      double funnelRollerVoltage,
      double intakeRollerVoltage,
      Elevator elevator,
      Manipulator manipulator,
      Funnel funnel,
      Intake intake) {
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
