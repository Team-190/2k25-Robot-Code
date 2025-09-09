package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoreSide;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorCSB;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.funnel.Funnel.FunnelCSB;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructure;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureStates;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A class that holds composite commands, which are sequences of commands for
 * complex robot actions.
 */
public class CompositeCommands {
  /**
   * A class that holds composite commands that are shared across different robot
   * versions.
   */
  public static final class SharedCommands {
    /**
     * Creates a command to reset the robot's heading to the alliance-specific zero.
     *
     * @param drive The drive subsystem.
     * @return A command to reset the heading.
     */
    public static final Command resetHeading(Drive drive) {
      return Commands.runOnce(
          () -> {
            RobotState.resetRobotPose(
                new Pose2d(
                    RobotState.getRobotPoseField().getTranslation(),
                    AllianceFlipUtil.apply(new Rotation2d())));
          })
          .ignoringDisable(true);
    }

    /**
     * Creates a command to set a static reef height in the robot state. This does
     * not move any
     * mechanisms.
     *
     * @param height The reef height to set.
     * @return A command to set the reef height.
     */
    public static final Command setStaticReefHeight(ReefState height) {
      return Commands.runOnce(() -> RobotState.setReefHeight(height));
    }
  }

  /** A class that holds composite commands for the V1_StackUp robot. */
  public static final class V1_StackUpCompositeCommands {
    /**
     * Creates a command to intake coral from the station.
     *
     * @param elevator    The elevator subsystem.
     * @param funnel      The funnel subsystem.
     * @param manipulator The manipulator subsystem.
     * @return A command to intake coral.
     */
    public static final Command intakeCoral(
        ElevatorCSB elevator, FunnelCSB funnel, V1_StackUpManipulator manipulator) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
          elevator.setPosition(() -> ReefState.CORAL_INTAKE),
          Commands.waitUntil(elevator::atGoal),
          Commands.race(
              manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())))
          .finallyDo(() -> RobotState.setIntakingCoral(false));
    }

    /**
     * Creates a command to intake coral from the station with an override.
     *
     * @param elevator    The elevator subsystem.
     * @param funnel      The funnel subsystem.
     * @param manipulator The manipulator subsystem.
     * @return A command to intake coral with an override.
     */
    public static final Command intakeCoralOverride(
        ElevatorCSB elevator, FunnelCSB funnel, V1_StackUpManipulator manipulator) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
          elevator.setPosition(() -> ReefState.CORAL_INTAKE),
          Commands.waitUntil(elevator::atGoal),
          Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
          .finallyDo(() -> RobotState.setIntakingCoral(false));
    }

    /**
     * Creates a command to score coral.
     *
     * @param manipulator The manipulator subsystem.
     * @return A command to score coral.
     */
    public static final Command scoreCoral(V1_StackUpManipulator manipulator) {
      return manipulator.scoreCoral().withTimeout(0.4);
    }

    /**
     * Creates a command sequence to score coral, waiting for auto-alignment.
     *
     * @param elevator    The elevator subsystem.
     * @param manipulator The manipulator subsystem.
     * @param autoAligned A supplier that returns true when the robot is aligned.
     * @return A command sequence to score coral.
     */
    public static final Command scoreCoralSequence(
        ElevatorCSB elevator, V1_StackUpManipulator manipulator, BooleanSupplier autoAligned) {
      return Commands.sequence(
          Commands.either(
              elevator.setPosition(() -> ReefState.L3),
              elevator.setPosition(),
              () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L4)
                  && !elevator.getPosition().equals(ElevatorPositions.L4)),
          Commands.waitUntil(() -> autoAligned.getAsBoolean()),
          elevator.setPosition(),
          Commands.waitSeconds(0.05),
          Commands.waitUntil(elevator::atGoal),
          Commands.either(
              manipulator.scoreL4Coral().withTimeout(0.4),
              manipulator.scoreCoral().withTimeout(0.15),
              () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L4)));
    }

    /**
     * Creates a command sequence to automatically score coral.
     *
     * @param drive       The drive subsystem.
     * @param elevator    The elevator subsystem.
     * @param manipulator The manipulator subsystem.
     * @param cameras     The vision cameras.
     * @return A command sequence to auto-score coral.
     */
    public static final Command autoScoreCoralSequence(
        Drive drive, ElevatorCSB elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
      return Commands.either(
          autoScoreL1CoralSequence(drive, elevator, manipulator, cameras),
          Commands.sequence(
              Commands.either(
                  elevator.setPosition(() -> ReefState.L2),
                  Commands.none(),
                  () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L1)
                      || RobotState.getOIData().currentReefHeight().equals(ReefState.STOW)
                      || RobotState.getOIData()
                          .currentReefHeight()
                          .equals(ReefState.CORAL_INTAKE)),
              Commands.parallel(
                  DriveCommands.autoAlignReefCoral(drive, cameras),
                  scoreCoralSequence(
                      elevator,
                      manipulator,
                      () -> RobotState.getReefAlignData().atCoralSetpoint())),
              elevator
                  .setPosition(() -> ReefState.STOW)
                  .onlyIf(
                      () -> elevator.getPosition().equals(ElevatorPositions.L3)
                          || elevator.getPosition().equals(ElevatorPositions.L2))),
          () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L1));
    }

    /**
     * Creates a command sequence to automatically score coral at L1.
     *
     * @param drive       The drive subsystem.
     * @param elevator    The elevator subsystem.
     * @param manipulator The manipulator subsystem.
     * @param cameras     The vision cameras.
     * @return A command sequence to auto-score coral at L1.
     */
    public static final Command autoScoreL1CoralSequence(
        Drive drive, ElevatorCSB elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefCoral(drive, cameras),
          scoreL1Coral(drive, elevator, manipulator));
    }

    /**
     * Creates a command to score coral at L1.
     *
     * @param drive       The drive subsystem.
     * @param elevator    The elevator subsystem.
     * @param manipulator The manipulator subsystem.
     * @return A command to score coral at L1.
     */
    public static final Command scoreL1Coral(
        Drive drive, ElevatorCSB elevator, V1_StackUpManipulator manipulator) {
      return Commands.sequence(
          elevator.setPosition(),
          Commands.waitSeconds(0.02),
          Commands.waitUntil(elevator::atGoal),
          Commands.parallel(
              manipulator.scoreL1Coral().withTimeout(0.8),
              Commands.sequence(
                  Commands.waitSeconds(0.05),
                  Commands.either(
                      DriveCommands.inchMovement(drive, -1, 0.1),
                      DriveCommands.inchMovement(drive, 1, 0.1),
                      () -> RobotState.getOIData().currentReefPost() == ReefPose.LEFT))));
    }

    /**
     * Creates a command for an emergency eject of coral.
     *
     * @param elevator    The elevator subsystem.
     * @param manipulator The manipulator subsystem.
     * @return A command to eject coral.
     */
    public static final Command emergencyEject(
        ElevatorCSB elevator, V1_StackUpManipulator manipulator) {
      return Commands.sequence(
          elevator.setPosition(() -> ReefState.L1),
          Commands.waitSeconds(0.125),
          Commands.waitUntil(elevator::atGoal),
          manipulator.scoreCoral().withTimeout(0.4),
          elevator.setPosition(() -> ReefState.STOW));
    }

    /**
     * Creates a command to remove algae from the reef. This uses the closest reef
     * tag to
     * automatically pick the reef height.
     *
     * @param drive       The drive subsystem.
     * @param elevator    The elevator subsystem.
     * @param manipulator The manipulator subsystem.
     * @param cameras     The vision cameras.
     * @return A command to remove algae.
     */
    public static final Command twerk(
        Drive drive, ElevatorCSB elevator, V1_StackUpManipulator manipulator, Camera... cameras) {
      return Commands.deferredProxy(
          () -> twerk(
              drive,
              elevator,
              manipulator,
              switch (RobotState.getReefAlignData().closestReefTag()) {
                case 10, 6, 8, 21, 17, 19 -> ReefState.ASS_BOT;
                case 9, 11, 7, 22, 20, 18 -> ReefState.ASS_TOP;
                default -> ReefState.ASS_BOT;
              },
              cameras));
    }

    /**
     * Creates a command to remove algae from the reef.
     *
     * @param drive       The drive subsystem.
     * @param elevator    The elevator subsystem.
     * @param manipulator The manipulator subsystem.
     * @param cameras     The vision cameras.
     * @return A command to remove algae.
     */
    public static final Command twerk(
        Drive drive,
        ElevatorCSB elevator,
        V1_StackUpManipulator manipulator,
        ReefState level,
        Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefAlgae(drive, cameras),
          elevator.setPosition(() -> ReefState.L4),
          Commands.waitUntil(elevator::atGoal),
          manipulator.toggleAlgaeArm(),
          Commands.waitSeconds(0.1),
          elevator.setPosition(() -> level),
          manipulator.removeAlgae().until(elevator::atGoal),
          manipulator.removeAlgae().withTimeout(0.35),
          manipulator.toggleAlgaeArm());
    }

    /**
     * Creates a command to set the dynamic reef height in the robot state. This
     * sets the height and
     * then moves the elevator to that position.
     *
     * @param height   The reef height to set.
     * @param elevator The elevator subsystem.
     * @return A command to set the dynamic reef height.
     */
    public static final Command setDynamicReefHeight(ReefState height, ElevatorCSB elevator) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setReefHeight(height)), elevator.setPosition());
    }

    /**
     * Creates a command to climb the robot.
     *
     * @param elevator The elevator subsystem.
     * @param funnel   The funnel subsystem.
     * @param climber  The climber subsystem.
     * @param drive    The drive subsystem.
     * @return A command to climb.
     */
    public static final Command climb(
        ElevatorCSB elevator, FunnelCSB funnel, Climber climber, Drive drive) {
      return Commands.sequence(
          elevator.setPosition(() -> ReefState.STOW),
          Commands.waitSeconds(0.02),
          Commands.waitUntil(elevator::atGoal),
          funnel.setClapDaddyGoal(FunnelState.CLIMB),
          Commands.parallel(
              climber.releaseClimber(),
              Commands.waitSeconds(
                  ClimberConstants.CLIMBER_TIMING_CONFIG.WAIT_AFTER_RELEASE_SECONDS())),
          Commands.waitUntil(climber::climberReady),
          Commands.deadline(climber.winchClimber(), Commands.run(drive::stop)));
    }
  }

  public static final class V2_RedundancyCompositeCommands {
    /**
     * Creates a command to intake coral from the station.
     *
     * @param superstructure The superstructure subsystem.
     * @param intake         The intake subsystem.
     * @return A command to intake coral.
     */
    public static final Command intakeCoralDriverSequence(
        V2_RedundancySuperstructure superstructure, V2_RedundancyIntake intake) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setHasAlgae(false)),
          superstructure.runGoalUntil(
              V2_RedundancySuperstructureStates.INTAKE_STATION, () -> intake.hasCoral()),
          superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN));
    }

    /**
     * Creates a command to intake coral from the station using the operator
     * sequence.
     *
     * @param superstructure The superstructure subsystem.
     * @param intake         The intake subsystem.
     * @return A command to intake coral using the operator sequence.
     */
    public static final Command intakeCoralOperatorSequence(
        V2_RedundancySuperstructure superstructure, V2_RedundancyIntake intake) {
      return Commands.sequence(
          superstructure.runGoalUntil(
              V2_RedundancySuperstructureStates.INTAKE_STATION, () -> intake.hasCoral()),
          superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN));
    }

    /**
     * Creates a command to score coral at L1.
     *
     * @param drive          The drive subsystem.
     * @param superstructure The superstructure subsystem.
     * @return A command to score coral at L1.
     */
    public static final Command scoreL1Coral(
        Drive drive, V2_RedundancySuperstructure superstructure) {
      return Commands.sequence(
          superstructure.runGoal(V2_RedundancySuperstructureStates.L1),
          Commands.parallel(
              superstructure.runReefScoreGoal(() -> ReefState.L1),
              Commands.sequence(
                  Commands.waitSeconds(0.05),
                  Commands.either(
                      DriveCommands.inchMovement(drive, -1, 0.1),
                      DriveCommands.inchMovement(drive, 1, 0.1),
                      () -> RobotState.getOIData().currentReefPost() == ReefPose.LEFT))));
    }

    /**
     * Creates a command sequence to score coral at L1, waiting for auto-alignment.
     *
     * @param drive          The drive subsystem.
     * @param superstructure The superstructure subsystem.
     * @return A command sequence to score coral at L1.
     */
    public static final Command autoScoreL1CoralSequence(
        Drive drive,
        ElevatorFSM elevator,
        V2_RedundancySuperstructure superstructure,
        Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefCoral(drive, cameras), scoreL1Coral(drive, superstructure));
    }

    /**
     * Creates a command sequence to score coral, waiting for auto-alignment.
     *
     * @param elevator       The elevator subsystem.
     * @param superstructure The superstructure subsystem.
     * @param autoAligned    A supplier that returns true when the robot is aligned.
     * @return A command sequence to score coral.
     */
    public static final Command scoreCoralSequence(
        ElevatorFSM elevator,
        V2_RedundancySuperstructure superstructure,
        BooleanSupplier autoAligned) {
      return Commands.sequence(
          Commands.either(
              superstructure.runGoal(V2_RedundancySuperstructureStates.L3),
              superstructure.runReefGoal(() -> RobotState.getOIData().currentReefHeight()),
              () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L4)
                  && !superstructure
                      .getCurrentState()
                      .equals(V2_RedundancySuperstructureStates.L4)),
          Commands.waitUntil(() -> autoAligned.getAsBoolean()),
          superstructure.runReefScoreGoal(() -> RobotState.getOIData().currentReefHeight()),
          superstructure
              .runGoal(V2_RedundancySuperstructureStates.STOW_DOWN)
              .onlyIf(
                  () -> elevator.getPosition().equals(ElevatorPositions.L3)
                      || elevator.getPosition().equals(ElevatorPositions.L2)));
    }

    /**
     * Creates a command sequence to automatically score coral.
     *
     * @param drive          The drive subsystem.
     * @param elevator       The elevator subsystem.
     * @param superstructure The superstructure subsystem.
     * @param cameras        The vision cameras.
     * @return A command sequence to auto-score coral.
     */
    public static final Command autoScoreCoralSequence(
        Drive drive,
        ElevatorFSM elevator,
        V2_RedundancySuperstructure superstructure,
        Camera... cameras) {

      return Commands.either(
          Commands.sequence(
              autoScoreL1CoralSequence(drive, elevator, superstructure, cameras),
              superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN)),
          Commands.sequence(
              Commands.either(
                  superstructure.runGoal(V2_RedundancySuperstructureStates.L2),
                  Commands.none(),
                  () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L1)
                      || RobotState.getOIData().currentReefHeight().equals(ReefState.STOW)
                      || RobotState.getOIData()
                          .currentReefHeight()
                          .equals(ReefState.CORAL_INTAKE)),
              Commands.parallel(
                  DriveCommands.autoAlignReefCoral(drive, cameras),
                  scoreCoralSequence(
                      elevator,
                      superstructure,
                      () -> RobotState.getReefAlignData().atCoralSetpoint())),
              superstructure
                  .l4PlusSequence()
                  .onlyIf(() -> RobotState.getOIData().currentReefHeight() == ReefState.L4)),
          () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L1));
    }

    /**
     * Creates a command to intake algae from the reef. This uses the closest reef
     * tag to
     * automatically pick the reef height and reef face.
     *
     * @param drive          The drive subsystem.
     * @param superstructure The superstructure subsystem.
     * @param cameras        The vision cameras.
     * @return A command to remove algae.
     */
    public static final Command intakeAlgaeFromReefSequence(
        Drive drive,
        V2_RedundancySuperstructure superstructure,
        Supplier<ReefState> level,
        Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefAlgae(drive, cameras),
          superstructure.runGoalUntil(
              () -> {
                switch (level.get()) {
                  case ALGAE_INTAKE_TOP:
                    return V2_RedundancySuperstructureStates.INTAKE_REEF_L3;
                  case ALGAE_INTAKE_BOTTOM:
                    return V2_RedundancySuperstructureStates.INTAKE_REEF_L2;
                  default:
                    return V2_RedundancySuperstructureStates.STOW_DOWN;
                }
              },
              () -> RobotState.isHasAlgae()),
          Commands.parallel(
              Commands.sequence(
                  Commands.waitSeconds(0.25),
                  Commands.either(
                      superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_UP),
                      superstructure.runGoal(
                          () -> {
                            switch (level.get()) {
                              case ALGAE_INTAKE_TOP:
                                return V2_RedundancySuperstructureStates.REEF_ACQUISITION_L3;
                              default:
                                return V2_RedundancySuperstructureStates.REEF_ACQUISITION_L2;
                            }
                          }),
                      () -> RobotState.isHasAlgae())),
              Commands.runEnd(
                  () -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), () -> drive.stop())
                  .withTimeout(0.5)));
    }

    /**
     * Creates a command to drop algae from the reef.
     *
     * @param drive          The drive subsystem.
     * @param elevator       The elevator subsystem.
     * @param manipulator    The manipulator subsystem.
     * @param intake         The intake subsystem.
     * @param superstructure The superstructure subsystem.
     * @param level          A supplier that provides the current reef level.
     * @param cameras        The vision cameras.
     * @return A command to drop algae from the reef.
     */
    public static final Command dropAlgae(
        Drive drive,
        ElevatorFSM elevator,
        V2_RedundancyManipulator manipulator,
        V2_RedundancyIntake intake,
        V2_RedundancySuperstructure superstructure,
        Supplier<ReefState> level,
        Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefAlgae(drive, cameras),
          Commands.sequence(
              superstructure
                  .runGoal(
                      () -> {
                        switch (level.get()) {
                          case ALGAE_INTAKE_TOP:
                            return V2_RedundancySuperstructureStates.INTAKE_REEF_L3;
                          case ALGAE_INTAKE_BOTTOM:
                            return V2_RedundancySuperstructureStates.INTAKE_REEF_L2;
                          default:
                            return V2_RedundancySuperstructureStates.STOW_DOWN;
                        }
                      })
                  .until(() -> RobotState.isHasAlgae()),
              Commands.waitSeconds(2.0),
              Commands.runEnd(
                  () -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), () -> drive.stop())
                  .withTimeout(0.5)),
          superstructure.runGoal(
              () -> {
                switch (level.get()) {
                  case ALGAE_INTAKE_TOP:
                    return V2_RedundancySuperstructureStates.DROP_REEF_L3;
                  case ALGAE_INTAKE_BOTTOM:
                    return V2_RedundancySuperstructureStates.DROP_REEF_L2;
                  default:
                    return V2_RedundancySuperstructureStates.STOW_DOWN;
                }
              }),
          Commands.waitSeconds(1.0),
          Commands.runOnce(() -> RobotState.setHasAlgae(false)),
          superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN));
    }

    /**
     * Creates a command sequence for the floor intake of algae.
     *
     * @param superstructure The superstructure subsystem.
     * @return A command sequence for the floor intake.
     */
    public static final Command floorIntakeSequence(V2_RedundancySuperstructure superstructure) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setHasAlgae(false)),
          superstructure.runGoalUntil(
              V2_RedundancySuperstructureStates.INTAKE_FLOOR, () -> RobotState.isHasAlgae()));
    }

    /**
     * Creates a command that posts the floor intake sequence, which can either go
     * up or down based
     * on whether the robot has algae.
     *
     * @param superstructure The superstructure subsystem.
     * @return A command that posts the floor intake sequence.
     */
    public static final Command postFloorIntakeSequence(
        V2_RedundancySuperstructure superstructure) {
      return Commands.either(
          superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_UP),
          superstructure.runGoal(V2_RedundancySuperstructureStates.STOW_DOWN),
          RobotState::isHasAlgae);
    }

    /**
     * Creates a command to set the dynamic reef height in the robot state. This
     * sets the height and
     * then moves the superstructure to that position.
     *
     * @param height         The reef height to set.
     * @param superstructure The superstructure subsystem.
     * @return A command to set the dynamic reef height.
     */
    public static final Command setDynamicReefHeight(
        ReefState height, V2_RedundancySuperstructure superstructure) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setReefHeight(height)), superstructure.setPosition());
    }

    /**
     * Creates a command to climb the robot.
     *
     * @param superstructure The superstructure subsystem.
     * @param climber        The climber subsystem.
     * @param drive          The drive subsystem.
     * @return A command to climb.
     */
    public static final Command climb(
        V2_RedundancySuperstructure superstructure, Climber climber, Drive drive) {
      return Commands.sequence(
          superstructure.runGoal(V2_RedundancySuperstructureStates.CLIMB),
          Commands.parallel(
              climber.releaseClimber(),
              Commands.waitSeconds(
                  ClimberConstants.CLIMBER_TIMING_CONFIG.WAIT_AFTER_RELEASE_SECONDS())),
          Commands.waitUntil(climber::climberReady),
          Commands.deadline(climber.winchClimber(), Commands.run(drive::stop)));
    }
  }

  /**
   * Creates a command sequence for homing all subsystems in the V2_Redundancy
   * robot.
   *
   * @param manipulator The manipulator subsystem.
   * @param intake      The intake subsystem.
   * @param elevator    The elevator subsystem.
   * @return A command sequence to home all subsystems.
   */
  public static final Command homingSequences(
      V2_RedundancyManipulator manipulator, V2_RedundancyIntake intake, ElevatorFSM elevator) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.setPosition(() -> ReefState.ALGAE_MID)),
        elevator.waitUntilAtGoal(),
        manipulator.homingSequence(),
        intake.homingSequence(),
        Commands.runOnce(() -> elevator.setPosition()));
  }

  public static final class V3_EpsilonCompositeCommands {

    public static final Command intakeCoralDriverSequence(
        V3_EpsilonSuperstructure superstructure,
        V3_EpsilonIntake intake,
        V3_EpsilonManipulator manipulator) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setHasAlgae(false)),
          superstructure.runGoalUntil(
              V3_EpsilonSuperstructureStates.GROUND_INTAKE, () -> intake.hasCoral()),
          Commands.waitSeconds(0.2),
          superstructure.runGoalUntil(
              V3_EpsilonSuperstructureStates.HANDOFF, () -> manipulator.hasCoral()));
      // superstructure.runGoal(V3_EpsilonSuperstructureStates.GROUND_INTAKE),
      // Commands.waitSeconds(0.2),
      // superstructure.runGoal(V3_EpsilonSuperstructureStates.HANDOFF));
    }

    /**
     * Creates a command to automatically align the robot to the optimal side of the
     * coral based on
     * the current reef height and the robot's orientation. This command sets the
     * score side in the
     * RobotState and then runs the auto-alignment command. After the command ends,
     * it resets the
     * score side to center.
     *
     * @param drive The drive subsystem.
     * @return A command to auto-align to the optimal side of the coral.
     */
    public static final Command optimalAutoAlignReefCoral(Drive drive) {
      return Commands.deadline(
          DriveCommands.autoAlignReefCoral(drive),
          Commands.run(
              () -> {
                if (RobotState.getOIData().currentReefHeight().equals(ReefState.L1)) {
                  RobotState.setScoreSide(ScoreSide.CENTER);
                } else if (Math.abs(
                    MathUtil.angleModulus(
                        RobotState.getReefAlignData()
                            .coralSetpoint()
                            .getRotation()
                            .rotateBy(Rotation2d.fromDegrees(-90))
                            .minus(RobotState.getRobotPoseField().getRotation())
                            .getRadians())) <= Math.abs(
                                MathUtil.angleModulus(
                                    RobotState.getReefAlignData()
                                        .coralSetpoint()
                                        .getRotation()
                                        .rotateBy(Rotation2d.fromDegrees(90))
                                        .minus(RobotState.getRobotPoseField().getRotation())
                                        .getRadians()))) {
                  RobotState.setScoreSide(ScoreSide.RIGHT);
                } else {
                  RobotState.setScoreSide(ScoreSide.LEFT);
                }
              }))
          .beforeStarting(() -> RobotState.setScoreSide(ScoreSide.CENTER));
    }

    public static final Command optimalAutoAlignReefAlgae(Drive drive, Camera... cameras) {
      return Commands.deadline(
          DriveCommands.autoAlignReefAlgae(drive, cameras),
          Commands.run(
              () -> {
                int closestReefTag = RobotState.getReefAlignData().closestReefTag();
                if (closestReefTag != -1) {
                  Pose2d baseAlgaeSetpoint = Reef.reefMap.get(closestReefTag).getAlgaeSetpoint();
                  if (Math.abs(
                      MathUtil.angleModulus(
                          baseAlgaeSetpoint
                              .getRotation()
                              .rotateBy(Rotation2d.fromDegrees(-90))
                              .minus(RobotState.getRobotPoseField().getRotation())
                              .getRadians())) <= Math.abs(
                                  MathUtil.angleModulus(
                                      baseAlgaeSetpoint
                                          .getRotation()
                                          .rotateBy(Rotation2d.fromDegrees(90))
                                          .minus(RobotState.getRobotPoseField().getRotation())
                                          .getRadians()))) {
                    RobotState.setScoreSide(ScoreSide.RIGHT);
                  } else {
                    RobotState.setScoreSide(ScoreSide.LEFT);
                  }
                }
              }))
          .beforeStarting(() -> RobotState.setScoreSide(ScoreSide.CENTER));
    }
  }
}
