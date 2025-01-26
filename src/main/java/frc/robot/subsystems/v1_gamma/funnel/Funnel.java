package frc.robot.subsystems.v1_gamma.funnel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.v1_gamma.funnel.FunnelConstants.FunnelState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs;
  private boolean isClosedLoop;
  private final SysIdRoutine serializerCharacterizationRoutine;
  private FunnelState goal;
  private boolean climbing;

  public Funnel(FunnelIO io) {
    this.io = io;
    inputs = new FunnelIOInputsAutoLogged();

    isClosedLoop = true;

    serializerCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Funnel/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setSerializerVoltage(volts.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);

    if (isClosedLoop) {
      setSerializerPosition(goal.getAngle());
    }

    if (climbing) {
      setSerializerGoal(FunnelState.CLIMB);
    } else {
      if (hasCoral()) {
        setSerializerGoal(FunnelState.CLOSED);
      } else {
        setSerializerGoal(FunnelState.OPENED);
      }
    }
  }

  /**
   * Sets the climbing state of the funnel.
   *
   * @param climbing True if the funnel is climbing, false otherwise.
   * @return A command to set the climbing state.
   */
  public Command setClimbing(boolean climbing) {
    return runOnce(() -> this.climbing = climbing);
  }

  /**
   * Sets the goal state of the serializer.
   *
   * @param goal The desired FunnelState.
   * @return A command to set the serializer goal.
   */
  public Command setSerializerGoal(FunnelState goal) {
    return runOnce(
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
  public Command setRollerVoltage(double volts) {
    return run(() -> io.setRollerVoltage(volts));
  }

  /**
   * Sets the voltage of the serializer.
   *
   * @param volts The desired voltage.
   * @return A command to set the serializer voltage.
   */
  public Command setSerializerVoltage(double volts) {
    isClosedLoop = false;
    return run(() -> io.setSerializerVoltage(volts));
  }

  /**
   * Sets the position of the serializer.
   *
   * @param radians The desired position in radians.
   * @return A command to set the serializer position.
   */
  public Command setSerializerPosition(double radians) {
    isClosedLoop = true;
    return run(() -> io.setSerializerPosition(Rotation2d.fromRadians(radians)));
  }

  /**
   * Stops the roller.
   *
   * @return A command to stop the roller.
   */
  public Command stopRoller() {
    return runOnce(io::stopRoller);
  }

  /**
   * Checks if the funnel has coral.
   *
   * @return True if the funnel has coral, false otherwise.
   */
  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  /**
   * Runs the SysId routine for the serializer.
   *
   * @return A command to run the SysId routine.
   */
  public Command sysIdRoutine() {
    return Commands.sequence(
        runOnce(() -> isClosedLoop = false),
        serializerCharacterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(4),
        serializerCharacterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(4),
        serializerCharacterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(4),
        serializerCharacterizationRoutine.dynamic(Direction.kReverse));
  }

  /**
   * Checks if the serializer motor is at the goal position.
   *
   * @return True if the serializer motor is at the goal, false otherwise.
   */
  @AutoLogOutput(key = "Funnel/Serializer Motor At Goal")
  public boolean serializerMotorAtGoal() {
    return io.atSerializerGoal();
  }

  /**
   * Updates the PID gains for the serializer.
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
   * Updates the angle thresholds for the serializer.
   *
   * @param maxAngle The maximum angle.
   * @param minAngle The minimum angle.
   */
  public void updateThresholds(double maxAngle, double minAngle) {
    io.updateThresholds(maxAngle, minAngle);
  }

  /**
   * Updates the motion constraints for the serializer.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  public void updateConstraints(double maxAcceleration, double maxVelocity) {
    io.updateConstraints(maxAcceleration, maxVelocity);
  }
}
