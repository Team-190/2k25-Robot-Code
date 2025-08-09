package frc.robot.subsystems.v3_Epsilon.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v3_Epsilon.intake.V3_EpsilonIntakeConstants.IntakeState;
import frc.robot.subsystems.v3_Epsilon.manipulator.V3_EpsilonManipulatorConstants;
import java.util.Set;
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

  private double holdVoltage() {
    double y;
    double x = Math.abs(inputs.rollerTorqueCurrentAmps);
    if (x <= 20) {
      y = -0.0003 * Math.pow(x, 3) + 0.0124286 * Math.pow(x, 2) - 0.241071 * x + 4.00643;
    } else {
      y = 0.0005 * Math.pow(x, 2) - 0.1015 * x + 3.7425;
    }
    return MathUtil.clamp(
        1.25 * y,
        0.10,
        V3_EpsilonManipulatorConstants.ROLLER_VOLTAGES.ALGAE_INTAKE_VOLTS().getAsDouble() / 1.5);
  }

  public void setRollerGoal(V3_EpsilonIntakeConstants.IntakeRollerStates rollerGoal) {
    if (hasCoral()
        && Set.of(
                V3_EpsilonManipulatorConstants.ManipulatorRollerStates.CORAL_INTAKE,
                V3_EpsilonManipulatorConstants.ManipulatorRollerStates.STOP)
            .contains(rollerGoal)) {

      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }
  }
}
