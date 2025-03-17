package frc.robot.subsystems.v2_Redundancy.inkake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v2_Redundancy.inkake.V2_RedundancyIntakeConstants.IntakeState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyIntake extends SubsystemBase {
  private final V2_RedundancyIntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;

  private final SysIdRoutine characterizationRoutine;
  private IntakeState goal;

  private boolean isClosedLoop;

  public V2_RedundancyIntake(V2_RedundancyIntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(1),
                (state) -> Logger.recordOutput("Intake/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setExtensionVoltage(volts.in(Volts)), null, this));
    goal = IntakeState.STOW;

    isClosedLoop = true;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (isClosedLoop) {
      io.setExtensionGoal(goal.getDistance());
    }
  }

  /**
   * Sets the goal state of the extension.
   *
   * @param goal The desired IntakeState.
   * @return A command to set the extension goal.
   */
  public Command setExtensionGoal(IntakeState goal) {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          this.goal = goal;
        });
  }

  /**
   * Sets the voltage of the roller.
   *
   * @param volts The desired voltage.
   * @return A command to set the roller voltage.
   */
  private Command setRollerVoltage(double volts) {
    return Commands.runEnd(() -> io.setRollerVoltage(volts), () -> io.setRollerVoltage(0));
  }

  /**
   * Stops the roller.
   *
   * @return A command to stop the roller.
   */
  public Command stopRoller() {
    return Commands.runOnce(io::stopRoller);
  }

  /**
   * Runs the SysId routine for the extension.
   *
   * @return A command to run the SysId routine.
   */
  public Command sysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> isClosedLoop = false),
        characterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(4),
        characterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(4),
        characterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(4),
        characterizationRoutine.dynamic(Direction.kReverse));
  }

  /**
   * Checks if the intake has algae.
   *
   * @return True if the intake has algae, false otherwise.
   */
  public boolean hasAlgae() {
    return false;
  }

  /**
   * Checks if the extension motor is at the goal position.
   *
   * @return True if the extension motor is at the goal, false otherwise.
   */
  @AutoLogOutput(key = "Intake/At Goal")
  public boolean atGoal() {
    return io.atExtensionPositionGoal();
  }

  public double getDistance() {
    return inputs.extensionPositionMeters;
  }

  /**
   * Updates the PID gains for the extension.
   *
   * @param kP The proportional gain.
   * @param kD The derivative gain.
   * @param kS The static gain.
   * @param kV The velocity gain.
   * @param kA The acceleration gain.
   */
  public void updateGains(double kP, double kD, double kS, double kV, double kA) {
    io.updateGains(kP, kD, kS, kV, kA);
  }

  /**
   * Updates the motion constraints for the extension.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    io.updateConstraints(maxAcceleration, maxVelocity);
  }

  public Command setIntakeVoltage(double volts) {
    return Commands.run(() -> io.setExtensionVoltage(volts));
  }

  public Command intakeAlgae(double volts) {
    return Commands.sequence(setExtensionGoal(IntakeState.INTAKE), setRollerVoltage(volts).until(() -> hasAlgae()));
} 

}
