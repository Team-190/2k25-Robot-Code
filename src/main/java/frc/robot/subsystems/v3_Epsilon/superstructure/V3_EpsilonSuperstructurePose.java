package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.shared.elevator.ElevatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorArmState;
import java.util.Optional;
import lombok.Getter;

/**
 * Represents a specific pose (configuration) of the superstructure, defining
 * the states of the
 * elevator, manipulator arm, intake, and funnel. This class allows for
 * coordinated control of these
 * subsystems to achieve a desired configuration.
 */
public class V3_EpsilonSuperstructurePose {
  private final String key;

  @Getter
  private final ReefState elevatorHeight;
  @Getter
  private final ManipulatorArmState armState;
  @Getter
  private final IntakePivotState intakeState;

  @Getter
  private final Optional<Double> flyByElevatorTolerance;
  @Getter
  private final Optional<Rotation2d> flyByArmTolerance;

  public V3_EpsilonSuperstructurePose(String key, SubsystemPoses poses) {
    this.key = key;

    this.elevatorHeight = poses.elevatorHeight();
    this.armState = poses.manipulatorArmState();
    this.intakeState = poses.intakePivotState();

    this.flyByElevatorTolerance = Optional.empty();
    this.flyByArmTolerance = Optional.empty();
  }

  public V3_EpsilonSuperstructurePose(String key, SubsystemPoses poses, double elevatorTolerance) {
    this.key = key;

    this.elevatorHeight = poses.elevatorHeight();
    this.armState = poses.manipulatorArmState();
    this.intakeState = poses.intakePivotState();

    this.flyByElevatorTolerance = Optional.of(elevatorTolerance);
    this.flyByArmTolerance = Optional.empty();
  }

  public V3_EpsilonSuperstructurePose(
      String key, SubsystemPoses poses, Rotation2d flyByArmTolerance) {
    this.key = key;

    this.elevatorHeight = poses.elevatorHeight();
    this.armState = poses.manipulatorArmState();
    this.intakeState = poses.intakePivotState();

    this.flyByElevatorTolerance = Optional.empty();
    this.flyByArmTolerance = Optional.of(flyByArmTolerance);
  }

  /**
   * Creates a command to set the elevator to the specified height for this pose.
   *
   * @param elevator The elevator subsystem to control.
   * @return A Command that sets the elevator height and waits until it reaches
   *         the goal.
   */
  public Command setElevatorHeight(ElevatorFSM elevator) {
    return Commands.parallel(Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)));
  }

  /**
   * Creates a command to set the intake to the specified extension state for this
   * pose.
   *
   * @param intake The intake subsystem to control.
   * @return A Command that sets the intake extension state and waits until it
   *         reaches the goal.
   */
  public Command setIntakeState(V3_EpsilonIntake intake) {
    return Commands.parallel(Commands.runOnce(() -> intake.setPivotGoal(intakeState)));
  }

  /**
   * Creates a command to set the manipulator arm to the specified state for this
   * pose.
   *
   * @param manipulator The manipulator subsystem to control.
   * @return A Command that sets the arm state and waits until it reaches the
   *         goal.
   */
  public Command setManipulatorState(V3_EpsilonManipulator manipulator) {
    return Commands.parallel(Commands.runOnce(() -> manipulator.setArmGoal(armState)));
  }

  /**
   * Creates a command that sets all subsystems (elevator, manipulator, intake,
   * and funnel) to the
   * states defined by this pose.
   *
   * @param elevator    The elevator subsystem.
   * @param intake      The intake subsystem.
   * @param manipulator The manipulator subsystem.
   * @return A Command that sets all subsystems to their respective states in
   *         parallel.
   */
  public Command asConfigurationSpaceCommand(
      ElevatorFSM elevator, V3_EpsilonIntake intake, V3_EpsilonManipulator manipulator) {
    return Commands.parallel(
        Commands.runOnce(() -> elevator.setPosition(() -> elevatorHeight)),
        Commands.runOnce(() -> manipulator.setArmGoal(armState)),
        Commands.runOnce(() -> intake.setPivotGoal(intakeState)));
  }

  public Command wait(
      ElevatorFSM elevator, V3_EpsilonIntake intake, V3_EpsilonManipulator manipulator) {

    if (flyByElevatorTolerance.isPresent()) {
      return Commands.waitUntil(() -> elevator.inTolerance(flyByElevatorTolerance.get()));
    } else if (flyByArmTolerance.isPresent()) {
      return Commands.waitUntil(() -> manipulator.armInTolerance(flyByArmTolerance.get()));
    } else {
      return Commands.parallel(
          elevator.waitUntilAtGoal(),
          manipulator.waitUntilArmAtGoal(),
          intake.waitUntilPivotAtGoal());
    }
  }

  /**
   * Returns a string representation of this pose (the key).
   *
   * @return The key of this pose.
   */
  public String toString() {
    return key;
  }

  /**
   * Computes the forward kinematics of the robot given the elevator and
   * manipulator states. This
   * method takes into account the offset of the elevator from the robot's base,
   * the position of the
   * elevator, and the angle of the manipulator's arm.
   *
   * @param elevator    The elevator subsystem.
   * @param manipulator The manipulator subsystem.
   * @return The computed forward kinematics of the robot as a Pose2d object.
   */
  public static Pose2d forwardKinematics(ElevatorFSM elevator, V3_EpsilonManipulator manipulator) {
    return new Pose2d()
        .plus(
            new Transform2d(
                0,
                ElevatorConstants.ELEVATOR_OFFSET_METERS,
                new Rotation2d())) // Y values should be elevator offsets
        .plus(new Transform2d(0.0, elevator.getPositionMeters(), new Rotation2d()))
        .plus(
            new Transform2d(0.0, 0.0, manipulator.getArmAngle().plus(new Rotation2d(Math.PI / 2))))
        .plus(
            new Transform2d(
                V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS(),
                0.0,
                new Rotation2d()));
  }

  /**
   * Computes the inverse kinematics of the robot given a translation from the
   * robot's base. This
   * method takes into account the offset of the elevator from the robot's base,
   * the position of the
   * elevator, and the angle of the manipulator's arm.
   * 
   * @param translation The translation from the robot's base to compute the
   *                    inverse kinematics for.
   * @return The computed inverse kinematics of the robot as a KineticsResult
   *         object.
   */
  public static KinematicsResult inverseKinematics(Translation2d translation) {
    return new KinematicsResult(
        translation.getY() - ElevatorConstants.ELEVATOR_OFFSET_METERS
            - Math.sqrt(Math.pow(V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS(), 2)
                - Math.pow(translation.getX(), 2)),
        new Rotation2d(translation.getX(),
            Math.sqrt(Math.pow(V3_EpsilonManipulatorConstants.ARM_PARAMETERS.LENGTH_METERS(), 2)
                - Math.pow(translation.getX(), 2))).minus(new Rotation2d(Math.PI/2)));
  }

  public record KinematicsResult(double elevatorHeightMeters, Rotation2d manipulatorAngle) {
  }

  /**
   * A record that groups the states of the elevator, manipulator arm, intake, and
   * funnel
   * subsystems. This is used to define a complete pose for the superstructure.
   */
  public record SubsystemPoses(
      ReefState elevatorHeight,
      ManipulatorArmState manipulatorArmState,
      IntakePivotState intakePivotState) {

    /**
     * Creates a SubsystemPoses instance with default states (STOW for elevator,
     * arm, and intake).
     */
    public SubsystemPoses() {
      this(ReefState.STOW, ManipulatorArmState.VERTICAL_UP, IntakePivotState.STOW);
    }
  }
}
