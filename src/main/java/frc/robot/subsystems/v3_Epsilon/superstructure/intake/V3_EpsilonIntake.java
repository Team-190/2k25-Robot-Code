package frc.robot.subsystems.v3_Epsilon.superstructure.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakeRollerState;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonIntake {
  private final V3_EpsilonIntakeIO io;
  private final V3_EpsilonIntakeIOInputsAutoLogged inputs;

  @Getter
  @AutoLogOutput(key = "Intake/Pivot Goal")
  private IntakePivotState pivotGoal;

  @Getter
  @AutoLogOutput(key = "Intake/Roller Goal")
  private IntakeRollerState rollerGoal;

  private boolean isClosedLoop;

  public V3_EpsilonIntake(V3_EpsilonIntakeIO io) {
    this.io = io;
    inputs = new V3_EpsilonIntakeIOInputsAutoLogged();

    pivotGoal = IntakePivotState.STOW;
    rollerGoal = IntakeRollerState.STOP;

    isClosedLoop = true;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (isClosedLoop) {
      io.setPivotGoal(pivotGoal.getAngle());
    }
  }

  @AutoLogOutput(key = "Intake/Has Coral")
  public boolean hasCoral() {
    return inputs.leftCANRangeDistanceMeters
            > V3_EpsilonIntakeConstants.INTAKE_CAN_CORAL_DETECTED_THRESHOLD_METERS
        && inputs.rightCANRangeDistanceMeters
            > V3_EpsilonIntakeConstants.INTAKE_CAN_CORAL_DETECTED_THRESHOLD_METERS;
  }

  @AutoLogOutput(key = "Intake/At Goal")
  public boolean pivotAtGoal() {
    return Math.abs(inputs.pivotPosition.getRadians() - pivotGoal.getAngle().getRadians())
        < V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.GOAL_TOLERANCE().getRadians();
  }

  public boolean pivotAtGoal(IntakePivotState goal) {
    return Math.abs(inputs.pivotPosition.getRadians() - goal.getAngle().getRadians())
        < V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.GOAL_TOLERANCE().getRadians();
  }

  public void setPivotGoal(IntakePivotState goal) {
    isClosedLoop = true;
    this.pivotGoal = goal;
  }

  public void setInnerRollerVoltage(double volts) {
    io.setInnerRollerVoltage(volts);
  }

  public void setOuterRollerVoltage(double volts) {
    io.setOuterRollerVoltage(volts);
  }

  public void setPivotVoltage(double volts) {
    isClosedLoop = false;
    io.setPivotVoltage(volts);
  }

  public Command waitUntilPivotAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::pivotAtGoal));
  }

  public void setRollerGoal(IntakeRollerState rollerGoal) {
    this.rollerGoal = rollerGoal;
  }

  public Rotation2d getPivotAngle() {
    return inputs.pivotPosition;
  }
}
