package frc.robot.subsystems.v2_Redundancy.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.Elevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.Funnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.Intake;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.Manipulator;

public class SuperstructureState {
  private final String key;
  protected final Elevator elevator;
  protected final Manipulator manipulator;
  protected final Funnel funnel;
  protected final Intake intake;

  public SuperstructureState(
      String key, Elevator elevator, Manipulator manipulator, Funnel funnel, Intake intake) {
    this.key = key;
    this.elevator = elevator;
    this.funnel = funnel;
    this.manipulator = manipulator;
    this.intake = intake;
  }

  public Command action() {
    return Commands.none();
  }

  public String toString() {
    return key;
  }
}
