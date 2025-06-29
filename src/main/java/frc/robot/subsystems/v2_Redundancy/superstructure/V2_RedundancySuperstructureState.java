package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;

public class V2_RedundancySuperstructureState {
  private final String key;
  protected final V2_RedundancyElevator elevator;
  protected final V2_RedundancyManipulator manipulator;
  protected final V2_RedundancyFunnel funnel;
  protected final V2_RedundancyIntake intake;

  public V2_RedundancySuperstructureState(
      String key,
      V2_RedundancyElevator elevator,
      V2_RedundancyManipulator manipulator,
      V2_RedundancyFunnel funnel,
      V2_RedundancyIntake intake) {
    this.key = key;
    this.elevator = elevator;
    this.funnel = funnel;
    this.manipulator = manipulator;
    this.intake = intake;
  }

  public Command asCommand() {
    return Commands.none();
  }

  public String toString() {
    return key;
  }
}
