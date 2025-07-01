package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberConstants;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.v1_StackUp.superstructure.elevator.V1_StackUpElevator;
import frc.robot.subsystems.v1_StackUp.superstructure.elevator.V1_StackUpElevatorConstants.V1_StackUpElevatorPositions;
import frc.robot.subsystems.v1_StackUp.superstructure.funnel.V1_StackUpFunnel;
import frc.robot.subsystems.v1_StackUp.superstructure.funnel.V1_StackUpFunnelConstants.FunnelState;
import frc.robot.subsystems.v1_StackUp.superstructure.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructure;
import frc.robot.subsystems.v2_Redundancy.superstructure.V2_RedundancySuperstructureStates.SuperstructureStates;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevatorConstants.V2_RedundancyElevatorPositions;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CompositeCommands {
  public static final class SharedCommands {
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

    public static final Command setStaticReefHeight(ReefState height) {
      return Commands.runOnce(() -> RobotState.setReefHeight(height));
    }
  }

  public static final class V1_StackUpCompositeCommands {
    public static final Command intakeCoral(
        V1_StackUpElevator elevator, V1_StackUpFunnel funnel, V1_StackUpManipulator manipulator) {
      return Commands.sequence(
              Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
              elevator.setPosition(() -> ReefState.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.race(
                  manipulator.intakeCoral(), funnel.intakeCoral(() -> manipulator.hasCoral())))
          .finallyDo(() -> RobotState.setIntakingCoral(false));
    }

    public static final Command intakeCoralOverride(
        V1_StackUpElevator elevator, V1_StackUpFunnel funnel, V1_StackUpManipulator manipulator) {
      return Commands.sequence(
              Commands.runOnce(() -> RobotState.setIntakingCoral(true)),
              elevator.setPosition(() -> ReefState.CORAL_INTAKE),
              Commands.waitUntil(elevator::atGoal),
              Commands.parallel(manipulator.intakeCoral(), funnel.intakeCoral(() -> false)))
          .finallyDo(() -> RobotState.setIntakingCoral(false));
    }

    public static final Command scoreCoral(V1_StackUpManipulator manipulator) {
      return manipulator.scoreCoral().withTimeout(0.4);
    }

    public static final Command scoreCoralSequence(
        V1_StackUpElevator elevator,
        V1_StackUpManipulator manipulator,
        BooleanSupplier autoAligned) {
      return Commands.sequence(
          Commands.either(
              elevator.setPosition(() -> ReefState.L3),
              elevator.setPosition(),
              () ->
                  RobotState.getOIData().currentReefHeight().equals(ReefState.L4)
                      && !elevator.getPosition().equals(V1_StackUpElevatorPositions.L4)),
          Commands.waitUntil(() -> autoAligned.getAsBoolean()),
          elevator.setPosition(),
          Commands.waitSeconds(0.05),
          Commands.waitUntil(elevator::atGoal),
          Commands.either(
              manipulator.scoreL4Coral().withTimeout(0.4),
              manipulator.scoreCoral().withTimeout(0.15),
              () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L4)));
    }

    public static final Command autoScoreCoralSequence(
        Drive drive,
        V1_StackUpElevator elevator,
        V1_StackUpManipulator manipulator,
        Camera... cameras) {
      return Commands.either(
          autoScoreL1CoralSequence(drive, elevator, manipulator, cameras),
          Commands.sequence(
              Commands.either(
                  elevator.setPosition(() -> ReefState.L2),
                  Commands.none(),
                  () ->
                      RobotState.getOIData().currentReefHeight().equals(ReefState.L1)
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
                      () ->
                          elevator.getPosition().equals(V1_StackUpElevatorPositions.L3)
                              || elevator.getPosition().equals(V1_StackUpElevatorPositions.L2))),
          () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L1));
    }

    public static final Command autoScoreL1CoralSequence(
        Drive drive,
        V1_StackUpElevator elevator,
        V1_StackUpManipulator manipulator,
        Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefCoral(drive, cameras),
          scoreL1Coral(drive, elevator, manipulator));
    }

    public static final Command scoreL1Coral(
        Drive drive, V1_StackUpElevator elevator, V1_StackUpManipulator manipulator) {
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

    public static final Command emergencyEject(
        V1_StackUpElevator elevator, V1_StackUpManipulator manipulator) {
      return Commands.sequence(
          elevator.setPosition(() -> ReefState.L1),
          Commands.waitSeconds(0.125),
          Commands.waitUntil(elevator::atGoal),
          manipulator.scoreCoral().withTimeout(0.4),
          elevator.setPosition(() -> ReefState.STOW));
    }

    public static final Command twerk(
        Drive drive,
        V1_StackUpElevator elevator,
        V1_StackUpManipulator manipulator,
        Camera... cameras) {
      return Commands.deferredProxy(
          () ->
              twerk(
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

    public static final Command twerk(
        Drive drive,
        V1_StackUpElevator elevator,
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

    public static final Command setDynamicReefHeight(
        ReefState height, V1_StackUpElevator elevator) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setReefHeight(height)), elevator.setPosition());
    }

    public static final Command climb(
        V1_StackUpElevator elevator, V1_StackUpFunnel funnel, Climber climber, Drive drive) {
      return Commands.sequence(
          elevator.setPosition(() -> ReefState.STOW),
          Commands.waitSeconds(0.02),
          Commands.waitUntil(elevator::atGoal),
          funnel.setClapDaddyGoal(FunnelState.CLIMB),
          Commands.parallel(
              climber.releaseClimber(),
              Commands.waitSeconds(ClimberConstants.WAIT_AFTER_RELEASE_SECONDS)),
          Commands.waitUntil(climber::climberReady),
          Commands.deadline(climber.winchClimber(), Commands.run(drive::stop)));
    }
  }

  public static final class V2_RedundancyCompositeCommands {
    public static final Command intakeCoralAuto(
        V2_RedundancySuperstructure superstructure, V2_RedundancyIntake intake) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setHasAlgae(false)),
          superstructure.runGoal(SuperstructureStates.INTAKE),
          Commands.waitUntil(() -> intake.hasCoral()),
          superstructure.runGoal(SuperstructureStates.STOW_DOWN));
    }

    public static final Command intakeCoralDriverSequence(
        V2_RedundancySuperstructure superstructure, V2_RedundancyIntake intake) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setHasAlgae(false)),
          superstructure.runGoalUntil(SuperstructureStates.INTAKE, () -> intake.hasCoral()),
          superstructure.runGoal(SuperstructureStates.STOW_DOWN));
    }

    public static final Command intakeCoralOperatorSequence(
        V2_RedundancySuperstructure superstructure, V2_RedundancyIntake intake) {
      return Commands.sequence(
          superstructure.runGoalUntil(SuperstructureStates.INTAKE, () -> intake.hasCoral()),
          superstructure.runGoal(SuperstructureStates.STOW_DOWN));
    }

    public static final Command scoreL1Coral(
        Drive drive, V2_RedundancySuperstructure superstructure) {
      return Commands.sequence(
          superstructure.runGoal(SuperstructureStates.L1),
          Commands.parallel(
              superstructure.runReefScoreGoal(() -> ReefState.L1),
              Commands.sequence(
                  Commands.waitSeconds(0.05),
                  Commands.either(
                      DriveCommands.inchMovement(drive, -1, 0.1),
                      DriveCommands.inchMovement(drive, 1, 0.1),
                      () -> RobotState.getOIData().currentReefPost() == ReefPose.LEFT))));
    }

    public static final Command autoScoreL1CoralSequence(
        Drive drive,
        V2_RedundancyElevator elevator,
        V2_RedundancySuperstructure superstructure,
        Camera... cameras) {
      return Commands.sequence(
          DriveCommands.autoAlignReefCoral(drive, cameras), scoreL1Coral(drive, superstructure));
    }

    public static final Command scoreCoralSequence(
        V2_RedundancyElevator elevator,
        V2_RedundancySuperstructure superstructure,
        BooleanSupplier autoAligned) {
      return Commands.sequence(
          Commands.either(
              superstructure.runGoal(SuperstructureStates.L3),
              superstructure.runReefGoal(() -> RobotState.getOIData().currentReefHeight()),
              () ->
                  RobotState.getOIData().currentReefHeight().equals(ReefState.L4)
                      && !superstructure.getCurrentState().equals(SuperstructureStates.L4)),
          Commands.waitUntil(() -> autoAligned.getAsBoolean()),
          superstructure.runReefScoreGoal(() -> RobotState.getOIData().currentReefHeight()),
          superstructure
              .runGoal(SuperstructureStates.STOW_DOWN)
              .onlyIf(
                  () ->
                      elevator.getPosition().equals(V2_RedundancyElevatorPositions.L3)
                          || elevator.getPosition().equals(V2_RedundancyElevatorPositions.L2)));
    }

    public static final Command autoScoreCoralSequence(
        Drive drive,
        V2_RedundancyElevator elevator,
        V2_RedundancySuperstructure superstructure,
        Camera... cameras) {

      return Commands.either(
          Commands.sequence(
              autoScoreL1CoralSequence(drive, elevator, superstructure, cameras),
              superstructure.runGoal(SuperstructureStates.STOW_DOWN)),
          Commands.sequence(
              Commands.either(
                  superstructure.runGoal(SuperstructureStates.L2),
                  Commands.none(),
                  () ->
                      RobotState.getOIData().currentReefHeight().equals(ReefState.L1)
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
                  .runReefScoreGoal(() -> ReefState.L4_PLUS)
                  .onlyIf(() -> superstructure.getCurrentState().equals(SuperstructureStates.L4))),
          () -> RobotState.getOIData().currentReefHeight().equals(ReefState.L1));
    }

    public static final Command intakeAlgaeFromReefSequence(
        Drive drive,
        V2_RedundancySuperstructure superstructure,
        Supplier<ReefState> level,
        Camera... cameras) {
      return Commands.sequence(
          superstructure
              .runGoal(
                  () -> {
                    switch (level.get()) {
                      case ALGAE_INTAKE_TOP:
                        return SuperstructureStates.INTAKE_REEF_L3;
                      case ALGAE_INTAKE_BOTTOM:
                        return SuperstructureStates.INTAKE_REEF_L2;
                      default:
                        return SuperstructureStates.STOW_DOWN;
                    }
                  })
              .until(() -> RobotState.isHasAlgae()),
          Commands.parallel(
              Commands.sequence(
                  Commands.waitSeconds(0.25),
                  Commands.either(
                      superstructure.runGoal(SuperstructureStates.STOW_UP),
                      Commands.none(),
                      () -> RobotState.isHasAlgae())),
              Commands.runEnd(
                      () -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), () -> drive.stop())
                  .withTimeout(0.5)));
    }

    public static final Command dropAlgae(
        Drive drive,
        V2_RedundancyElevator elevator,
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
                            return SuperstructureStates.INTAKE_REEF_L3;
                          case ALGAE_INTAKE_BOTTOM:
                            return SuperstructureStates.INTAKE_REEF_L2;
                          default:
                            return SuperstructureStates.STOW_DOWN;
                        }
                      })
                  .until(() -> RobotState.isHasAlgae()),
              superstructure
                  .runGoal(
                      () -> {
                        switch (level.get()) {
                          case ALGAE_INTAKE_TOP:
                            return SuperstructureStates.REEF_ACQUISITION_L3;
                          case ALGAE_INTAKE_BOTTOM:
                            return SuperstructureStates.REEF_ACQUISITION_L2;
                          default:
                            return SuperstructureStates.STOW_DOWN;
                        }
                      })
                  .withTimeout(0.02),
              Commands.waitSeconds(1.0),
              Commands.runEnd(
                      () -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), () -> drive.stop())
                  .withTimeout(0.5)),
          superstructure
              .runGoal(
                  () -> {
                    switch (level.get()) {
                      case ALGAE_INTAKE_TOP:
                        return SuperstructureStates.DROP_REEF_L3;
                      case ALGAE_INTAKE_BOTTOM:
                        return SuperstructureStates.DROP_REEF_L2;
                      default:
                        return SuperstructureStates.STOW_DOWN;
                    }
                  })
              .withTimeout(0.75),
          superstructure.runGoal(SuperstructureStates.STOW_DOWN));
    }

    public static final Command floorIntakeSequence(V2_RedundancySuperstructure superstructure) {
      return Commands.sequence(
          Commands.parallel(
              superstructure.runGoal(SuperstructureStates.FLOOR_ACQUISITION),
              Commands.runOnce(() -> RobotState.setHasAlgae(false))),
          superstructure.runGoalUntil(
              SuperstructureStates.INTAKE_FLOOR, () -> RobotState.isHasAlgae()));
    }

    public static final Command postFloorIntakeSequence(
        V2_RedundancySuperstructure superstructure) {
      return Commands.either(
          superstructure.runGoal(SuperstructureStates.STOW_UP),
          superstructure.runGoal(SuperstructureStates.STOW_DOWN),
          RobotState::isHasAlgae);
    }

    public static final Command setDynamicReefHeight(
        ReefState height, V2_RedundancySuperstructure superstructure) {
      return Commands.sequence(
          Commands.runOnce(() -> RobotState.setReefHeight(height)), superstructure.setPosition());
    }

    public static final Command climb(
        V2_RedundancySuperstructure superstructure, Climber climber, Drive drive) {
      return Commands.sequence(
          superstructure.runGoal(SuperstructureStates.CLIMB),
          Commands.parallel(
              climber.releaseClimber(),
              Commands.waitSeconds(ClimberConstants.WAIT_AFTER_RELEASE_SECONDS)),
          Commands.waitUntil(climber::climberReady),
          Commands.deadline(climber.winchClimber(), Commands.run(drive::stop)));
    }
  }

  public static final Command homingSequences(
      V2_RedundancyManipulator manipulator,
      V2_RedundancyIntake intake,
      V2_RedundancyElevator elevator) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.setPosition(() -> ReefState.ALGAE_MID)),
        elevator.waitUntilAtGoal(),
        manipulator.homingSequence(),
        intake.homingSequence(),
        Commands.runOnce(() -> elevator.setPosition()));
  }
}
