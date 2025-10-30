package frc.robot.subsystems.v3_Epsilon.superstructure.manipulator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructure;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureStates;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorArmState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorRollerState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.Side;
import java.util.Set;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonManipulator {
  private final V3_EpsilonManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs;

  @AutoLogOutput(key = "Manipulator/Arm Goal")
  @Getter
  private Rotation2d armGoal;

  @Setter
  @Getter
  @AutoLogOutput(key = "Manipulator/Arm Side")
  private Side armSide;

  @AutoLogOutput(key = "Manipulator/Roller Goal")
  @Getter
  private ManipulatorRollerState rollerGoal;

  private boolean isClosedLoop;

  @Setter @Getter private boolean clearsElevator;

  public V3_EpsilonManipulator(V3_EpsilonManipulatorIO io) {
    this.io = io;
    inputs = new ManipulatorIOInputsAutoLogged();

    isClosedLoop = true;
    armGoal = ManipulatorArmState.VERTICAL_UP.getAngle(armSide);
    armSide = Side.POSITIVE;
    rollerGoal = ManipulatorRollerState.STOP;

    clearsElevator = false;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);

    if (isClosedLoop) {
      Rotation2d goal = armGoal;

      if (!isSafePosition() || clearsElevator) {
        if (armSide == Side.POSITIVE) {
          goal = Rotation2d.fromRotations(goal.getRotations() - 1.0);
        } else {
          goal = Rotation2d.fromRotations(goal.getRotations() + 1.0);
        }
      }

      io.setArmGoal(goal);
    }

    if (rollerGoal.equals(ManipulatorRollerState.SCORE_ALGAE)) {
      io.setRollerVoltage(ManipulatorRollerState.SCORE_ALGAE.getVoltage());
    } else if (hasAlgae()
        && Set.of(
                ManipulatorRollerState.CORAL_INTAKE,
                ManipulatorRollerState.STOP,
                ManipulatorRollerState.ALGAE_INTAKE)
            .contains(rollerGoal)
        && !rollerGoal.equals(ManipulatorRollerState.SCORE_ALGAE)) {
      RobotState.setHasAlgae(true);
      io.setRollerVoltage(holdVoltage());
    } else if (hasCoral()) {
      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }
  }

  /**
   * Checks if the manipulator is currently detecting coral. This is done by checking if the CAN
   * range sensor is detecting a distance less than the coral detection threshold and greater than
   * 0.
   *
   * @return True if the manipulator is detecting coral, false otherwise.
   */
  @AutoLogOutput(key = "Manipulator/Has Coral")
  public boolean hasCoral() {
    return inputs.canRangeGot;
  }

  /**
   * Checks if the manipulator is currently detecting algae. This is done by checking if the CAN
   * range sensor is detecting a distance less than the algae detection threshold and greater than
   * 0.
   *
   * @return True if the manipulator is detecting algae, false otherwise.
   */
  @AutoLogOutput(key = "Manipulator/Has Algae")
  public boolean hasAlgae() {
    boolean hasAlgae = rollerGoal.equals(ManipulatorRollerState.ALGAE_INTAKE) && inputs.canRangeGot;
    return hasAlgae;
  }

  /**
   * Creates a command to run the manipulator arm at a specified voltage.
   *
   * @param volts The voltage to set the arm to.
   * @return A command to run the arm.
   */
  public Command runArm(double volts) {
    return Commands.runEnd(
        () -> {
          isClosedLoop = false;
          io.setArmVoltage(volts);
        },
        () -> io.setArmVoltage(0));
  }

  /**
   * Sets the goal for the manipulator arm to reach.
   *
   * @param goal The goal state to set the arm to.
   */
  public void setArmGoal(ManipulatorArmState goal) {
    isClosedLoop = true;
    armGoal = goal.getAngle(armSide);
  }

  public void setArmGoal(Rotation2d goal) {
    isClosedLoop = true;
    armGoal = goal;
  }

  /**
   * Updates the gains for the manipulator arm. This function sets the PID gains for the three slots
   * of the arm. The gains are used to control the arm's movement.
   *
   * @param kP0 The proportional gain for slot 0.
   * @param kD0 The derivative gain for slot 0.
   * @param kS0 The static gain for slot 0.
   * @param kV0 The velocity gain for slot 0.
   * @param kA0 The acceleration gain for slot 0.
   * @param kG0 The gravity gain for slot 0.
   * @param kP1 The proportional gain for slot 1.
   * @param kD1 The derivative gain for slot 1.
   * @param kS1 The static gain for slot 1.
   * @param kV1 The velocity gain for slot 1.
   * @param kA1 The acceleration gain for slot 1.
   * @param kG1 The gravity gain for slot 1.
   * @param kP2 The proportional gain for slot 2.
   * @param kD2 The derivative gain for slot 2.
   * @param kS2 The static gain for slot 2.
   * @param kV2 The velocity gain for slot 2.
   * @param kA2 The acceleration gain for slot 2.
   * @param kG2 The gravity gain for slot 2.
   */
  public void updateArmGains(
      double kP0,
      double kD0,
      double kS0,
      double kV0,
      double kA0,
      double kG0,
      double kP1,
      double kD1,
      double kS1,
      double kV1,
      double kA1,
      double kG1,
      double kP2,
      double kD2,
      double kS2,
      double kV2,
      double kA2,
      double kG2) {
    io.updateSlot0ArmGains(kP0, kD0, kS0, kV0, kA0, kG0);
    io.updateSlot1ArmGains(kP1, kD1, kS1, kV1, kA1, kG1);
    io.updateSlot2ArmGains(kP2, kD2, kS2, kV2, kA2, kG2);
  }

  /**
   * Updates the constraints for the arm.
   *
   * @param maxAcceleration The maximum acceleration.
   * @param maxVelocity The maximum velocity.
   */
  public void updateArmConstraints(double maxAcceleration, double maxVelocity) {
    io.updateArmConstraints(maxAcceleration, maxVelocity);
  }

  /**
   * Checks if the arm is at the goal position.
   *
   * <p>This function checks if the arm is within the goal tolerance of the currently set arm goal
   * position. If the arm is within the tolerance, it returns true. Otherwise, it returns false.
   *
   * @return If the arm is at the goal position.
   */
  @AutoLogOutput(key = "Manipulator/Arm At Goal")
  public boolean armAtGoal() {
    return armAtGoal(armGoal);
  }

  public boolean armAtGoal(Rotation2d state) {
    return Math.abs(inputs.armPosition.minus(state).getRadians())
        <= V3_EpsilonManipulatorConstants.CONSTRAINTS.goalToleranceRadians().get();
  }

  public boolean armInTolerance(Rotation2d tolerance) {
    return Math.abs(inputs.armPosition.minus(armGoal).getRadians()) <= tolerance.getRadians();
  }
  /**
   * Waits until the arm is at the goal position.
   *
   * <p>This command waits for 0.02 seconds and then checks if the arm is at the goal position. If
   * the arm is not at the goal position, it waits for 0.02 seconds and checks again. This process
   * repeats until the arm is at the goal position.
   *
   * @return A command that waits until the arm is at the goal position.
   */
  public Command waitUntilArmAtGoal() {
    return Commands.sequence(Commands.waitSeconds(0.02), Commands.waitUntil(this::armAtGoal));
  }

  /**
   * Returns a voltage to hold the roller at the current position. The voltage is calculated based
   * on the current torque current of the roller. The calculation is done using a piecewise
   * polynomial function. The function first checks if the current torque current is less than or
   * equal to 20. If it is, it uses one set of coefficients to calculate the voltage. Otherwise, it
   * uses another set of coefficients.
   */
  private double holdVoltage() {
    return hasAlgae() ? -12.0 : -1;
  }

  /**
   * Sets the current slot of the manipulator arm based on the current state of the subsystem. If
   * the subsystem has algae, it sets the slot to 2. If the subsystem has coral, it sets the slot to
   * 1. Otherwise, it sets the slot to 0.
   */
  public void setSlot() {
    if (hasAlgae()) {
      io.setSlot(2);
    } else if (hasCoral()) {
      io.setSlot(1);
    } else {
      io.setSlot(0);
    }
  }

  /**
   * Creates a command to run the SysId routine for the manipulator arm, generating the constants
   * and gains for a PID.
   *
   * @param superstructure The V3 Epsiolon Superstructure.
   * @return A command to run the SysId routine for the manipulator arm.
   */
  public Command sysIdRoutine(V3_EpsilonSuperstructure superstructure, ElevatorFSM elevator) {
    SysIdRoutine algaeCharacterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(8),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Manipulator/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> io.setArmVoltage(volts.in(Volts)), null, superstructure));
    return Commands.sequence(
        superstructure.runGoal(V3_EpsilonSuperstructureStates.OVERRIDE),
        Commands.runOnce(() -> elevator.setPosition(() -> ReefState.L4)),
        Commands.runOnce(() -> isClosedLoop = false),
        algaeCharacterizationRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(.25),
        algaeCharacterizationRoutine.dynamic(Direction.kReverse));
  }
  /**
   * Sets the manipulator arm to the specified state.
   *
   * @param state The state to set the arm to.
   */
  public void setManipulatorState(V3_EpsilonManipulatorConstants.ManipulatorArmState state) {
    io.setManipulatorState(state);
  }

  /**
   * Sets the roller goal state of the manipulator. If the subsystem has algae or coral and the goal
   * is one of the intake states, it sets the roller voltage to the hold voltage. Otherwise, it sets
   * the roller voltage to the goal voltage.
   *
   * @param rollerGoal The desired state of the roller.
   */
  public void setRollerGoal(V3_EpsilonManipulatorConstants.ManipulatorRollerState rollerGoal) {
    this.rollerGoal = rollerGoal;
    if ((hasAlgae() || hasCoral())
        && Set.of(
                V3_EpsilonManipulatorConstants.ManipulatorRollerState.ALGAE_INTAKE,
                V3_EpsilonManipulatorConstants.ManipulatorRollerState.CORAL_INTAKE,
                V3_EpsilonManipulatorConstants.ManipulatorRollerState.STOP)
            .contains(rollerGoal)) {

      io.setRollerVoltage(holdVoltage());
    } else {
      io.setRollerVoltage(rollerGoal.getVoltage());
    }
  }

  /**
   * Gets the current angle of the manipulator arm.
   *
   * @return The current angle of the manipulator arm, in radians.
   */
  public Rotation2d getArmAngle() {
    return inputs.armPosition;
  }

  /**
   * Checks if the manipulator arm is currently in a safe position. A safe position is when the arm
   * is pointing away from the robot's body. The safe position threshold is determined by the angle
   * between the arm and the robot's body. If the angle is greater than the threshold, it is
   * considered safe.
   *
   * @return true if the arm is in a safe position, false otherwise.
   */
  @AutoLogOutput(key = "Manipulator/Safe Position")
  public boolean isSafePosition() {
    double cosThresh =
        Math.cos(Math.PI - ManipulatorArmState.SAFE_ANGLE.getAngle(armSide).getRadians());
    // unsafe if -cos(theta) >= cosThresh
    return (-inputs.armPosition.getCos()) < cosThresh;
  }

  public double getArmVelocity() {
    return inputs.armVelocityRadiansPerSecond;
  }

  public double getRollerVelocity() {
    return inputs.rollerVelocityRadiansPerSecond;
  }
}
