package frc.robot.subsystems.v2_Redundancy.manipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyManipulator extends SubsystemBase {
  private final V2_RedundancyManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;
  private boolean isClosedLoop;
  private final SysIdRoutine algaeCharacterizationRoutine;
  private final Timer currentTimer;

  public V2_RedundancyManipulator(V2_RedundancyManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();
    isClosedLoop = true;
    algaeCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(5),
                (state) -> Logger.recordOutput("Manipulator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setArmVoltage(volts.in(Volts)), null, this));

    currentTimer = new Timer();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

    if (isClosedLoop) io.setArmPositionGoal(new Rotation2d());
  }

  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return Math.abs(inputs.rollerTorqueCurrentAmps)
        > V2_RedundancyManipulatorConstants.ROLLER_CURRENT_THRESHOLD;
  }

  @AutoLogOutput(key = "Manipulator/Has Algae")
  public boolean hasAlgae() {
    return Math.abs(inputs.rollerTorqueCurrentAmps)
        > V2_RedundancyManipulatorConstants.ROLLER_CURRENT_THRESHOLD;
  }

  public Command runManipulator(double volts) {
    return Commands.runEnd(() -> io.setRollerVoltage(volts), () -> io.setRollerVoltage(0));
  }

  public Command intakeCoral() {
    return Commands.sequence(
        Commands.runOnce(() -> currentTimer.restart()),
        runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.INTAKE_VOLTS().get())
            .until(() -> hasCoral() && currentTimer.hasElapsed(0.25)));
  }

  public Command intakeAlgae() {
    return Commands.sequence(
        Commands.runOnce(() -> currentTimer.restart()),
        runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.INTAKE_VOLTS().get())
            .until(() -> hasAlgae() && currentTimer.hasElapsed(0.25)),
        Commands.runOnce(() -> RobotState.setHasAlgae(true)));
  }

  public Command scoreCoral() {
    return runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_VOLTS().get());
  }

  public Command scoreAlgae() {
    return Commands.sequence(
        runManipulator(-V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.SCORE_VOLTS().get()),
        Commands.runOnce(() -> RobotState.setHasAlgae(false)));
  }

  public Command scoreL1Coral() {
    return runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.L1_VOLTS().get());
  }

  public Command halfScoreCoral() {
    return runManipulator(V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.HALF_VOLTS().get());
  }

  public Command unHalfScoreCoral() {
    return runManipulator(-V2_RedundancyManipulatorConstants.ROLLER_VOLTAGES.HALF_VOLTS().get());
  }

  public Command sysIdRoutine() {
    return Commands.parallel(
        runManipulator(-0.5),
        Commands.sequence(
            Commands.runOnce(() -> isClosedLoop = false),
            algaeCharacterizationRoutine.quasistatic(Direction.kForward),
            Commands.waitSeconds(4),
            algaeCharacterizationRoutine.quasistatic(Direction.kReverse),
            Commands.waitSeconds(4),
            algaeCharacterizationRoutine.dynamic(Direction.kForward),
            Commands.waitSeconds(4),
            algaeCharacterizationRoutine.dynamic(Direction.kReverse)));
  }
}
