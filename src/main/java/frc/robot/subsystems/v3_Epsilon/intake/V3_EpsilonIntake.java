package frc.robot.subsystems.v3_Epsilon.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants.IntakeState;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonIntake extends SubsystemBase {
  private final V3_EpsilonIntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final SysIdRoutine characterizationRoutine;

  @Getter
  @AutoLogOutput(key = "Intake/Goal")
  private IntakeState goal;

  private boolean isClosedLoop;

  public V3_EpsilonIntake(V3_EpsilonIntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(8),
                (state) -> Logger.recordOutput("Intake/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setPivotVoltage(volts.in(Volts)), null, this));

    isClosedLoop = true;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (isClosedLoop) {
      io.setPivotGoal(goal.getAngle());
    }
  }

  @AutoLogOutput(key = "Intake/Has Coral")
  public boolean hasCoral() {
    return false; // Udpate later
  }

  @AutoLogOutput(key = "Intake/At Goal")
  public boolean atGoal() {
    return Math.abs(inputs.pivotPosition.getRadians() - goal.getAngle().getRadians())
        < V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.GOAL_TOLERANCE().getRadians();
  }

  public void waitUntilIntakeAtGoal() {
    Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::atGoal));
  }

  public void setGoal(IntakeState goal) {
    isClosedLoop = true;
    this.goal = goal;
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
}
