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
  // private final SysIdRoutine characterizationRoutine;

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

    // characterizationRoutine =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             Volts.of(0.2).per(Second),
    //             Volts.of(3.5),
    //             Seconds.of(8),
    //             (state) -> Logger.recordOutput("Intake/SysID State", state.toString())),
    //         new SysIdRoutine.Mechanism((volts) -> io.setPivotVoltage(volts.in(Volts)), null,
    // this));

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
    return false; // Udpate later
  }

  @AutoLogOutput(key = "Intake/At Goal")
  public boolean atGoal() {
    return Math.abs(inputs.pivotPosition.getRadians() - pivotGoal.getAngle().getRadians())
        < V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.GOAL_TOLERANCE().getRadians();
  }

  public void setPivotGoal(IntakePivotState goal) {
    isClosedLoop = true;
    this.pivotGoal = goal;
  }

  public void setRollerVoltage(double volts) {
    io.setRollerVoltage(volts);
  }

  public void setPivotVoltage(double volts) {
    isClosedLoop = false;
    io.setPivotVoltage(volts);
  }

  public Command waitUntilPivotAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::atGoal));
  }

  public Command sysIdRoutine() {
    return Commands.sequence();
    // Commands.runOnce(() -> isClosedLoop = false),
    // Commands.runOnce(() -> characterizationRoutine.quasistatic(Direction.kForward)),
    // Commands.waitSeconds(0.25),
    // Commands.runOnce(() -> characterizationRoutine.quasistatic(Direction.kReverse)),
    // Commands.waitSeconds(0.25),
    // Commands.runOnce(() -> characterizationRoutine.dynamic(Direction.kForward)),
    // Commands.waitSeconds(0.25),
    // Commands.runOnce(() -> characterizationRoutine.dynamic(Direction.kReverse)));
  }

  public void setRollerGoal(IntakeRollerState rollerGoal) {
    this.rollerGoal = rollerGoal;
  }

  public Rotation2d getPivotAngle() {
    return inputs.pivotPosition;
  }
}
