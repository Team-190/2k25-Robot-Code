package frc.robot.subsystems.v3_Epsilon.superstructure.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructure;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakeRollerStates;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakeState;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonIntake {
  private final V3_EpsilonIntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;

  @Getter
  @AutoLogOutput(key = "Intake/Pivot Goal")
  private IntakeState pivotGoal;

  @Getter
  @AutoLogOutput(key = "Intake/Roller Goal")
  private IntakeRollerStates rollerGoal = IntakeRollerStates.STOP;

  private boolean isClosedLoop;

  public V3_EpsilonIntake(V3_EpsilonIntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();

    isClosedLoop = true;
    pivotGoal = IntakeState.STOW;
    rollerGoal = IntakeRollerStates.STOP;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (isClosedLoop) {
      io.setPivotGoal(pivotGoal.getAngle());
    }
    io.setRollerVoltage(rollerGoal.getVoltage());
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

  public void waitUntilIntakeAtGoal() {
    Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::atGoal));
  }

  public void setPivotGoal(IntakeState goal) {
    isClosedLoop = true;
    this.pivotGoal = goal;
  }

  public void setPivotVoltage(double volts) {
    isClosedLoop = false;
    io.setPivotVoltage(volts);
  }

  public Command waitUntilPivotAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::atGoal));
  }

  public Command sysIdRoutine(V3_EpsilonSuperstructure superstructure) {
    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(8),
                (state) -> Logger.recordOutput("Intake/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setPivotVoltage(volts.in(Volts)), null, superstructure));
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        Commands.runOnce(() -> characterizationRoutine.quasistatic(Direction.kForward)),
        Commands.waitSeconds(0.25),
        Commands.runOnce(() -> characterizationRoutine.quasistatic(Direction.kReverse)),
        Commands.waitSeconds(0.25),
        Commands.runOnce(() -> characterizationRoutine.dynamic(Direction.kForward)),
        Commands.waitSeconds(0.25),
        Commands.runOnce(() -> characterizationRoutine.dynamic(Direction.kReverse)));
  }

  public void setRollerGoal(V3_EpsilonIntakeConstants.IntakeRollerStates rollerGoal) {
    this.rollerGoal = rollerGoal;
  }

  public Rotation2d getPivotAngle() {
    return inputs.pivotPosition;
  }
}
