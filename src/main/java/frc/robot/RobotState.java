package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.shared.vision.CameraDuty;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeometryUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  @Setter
  @Getter
  @AutoLogOutput(key = "RobotState/Reef Data/Current Reef Post")
  private static ReefPost currentReefPost = ReefPost.LEFT;

  @Getter private static int closestReefTag = -1;
  @Getter private static Pose2d setpoint = new Pose2d();
  @Getter private static double distanceToPost = Double.POSITIVE_INFINITY;
  @Getter private static Boolean atThreshold = false;

  private static final SwerveDrivePoseEstimator fieldLocalizer;
  private static final SwerveDrivePoseEstimator reefLocalizer;
  private static final SwerveDriveOdometry odometry;

  private static Rotation2d robotHeading;
  private static Rotation2d headingOffset;
  private static SwerveModulePosition[] modulePositions;

  static {
    switch (Constants.ROBOT) {
      case V0_FUNKY:
      case V0_FUNKY_SIM:
        break;
      case V0_WHIPLASH:
      case V0_WHIPLASH_SIM:
        break;
      case V1_GAMMA:
      case V1_GAMMA_SIM:
        break;
      case V2_DELTA:
      case V2_DELTA_SIM:
        break;
    }

    modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    fieldLocalizer =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_CONFIG.kinematics(),
            new Rotation2d(),
            modulePositions,
            new Pose2d());
    reefLocalizer =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_CONFIG.kinematics(),
            new Rotation2d(),
            modulePositions,
            new Pose2d());
    odometry =
        new SwerveDriveOdometry(
            DriveConstants.DRIVE_CONFIG.kinematics(), new Rotation2d(), modulePositions);

    headingOffset = new Rotation2d();
  }

  public RobotState() {}

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      Translation2d robotFieldRelativeVelocity,
      SwerveModulePosition[] modulePositions,
      Camera[] cameras) {

    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;

    odometry.update(robotHeading, modulePositions);
    reefLocalizer.updateWithTime(Timer.getFPGATimestamp(), robotHeading, modulePositions);
    fieldLocalizer.updateWithTime(Timer.getFPGATimestamp(), robotHeading, modulePositions);

    for (Camera camera : cameras) {
      double[] limelightHeadingData = {
        robotHeading.minus(headingOffset).getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
      };
      camera.getRobotHeadingPublisher().set(limelightHeadingData, latestRobotHeadingTimestamp);
    }
    NetworkTableInstance.getDefault().flush();

    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.FIELD_LOCALIZATION)
          && camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && !GeometryUtil.isZero(camera.getSecondaryPose())
          && Math.abs(robotYawVelocity) <= Units.degreesToRadians(15.0)
          && Math.abs(robotFieldRelativeVelocity.getNorm()) <= 1.0) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        fieldLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
        if (camera.getTotalTargets() > 1) {
          double xyStddevSecondary =
              camera.getSecondaryXYStandardDeviationCoefficient()
                  * Math.pow(camera.getAverageDistance(), 2.0)
                  / camera.getTotalTargets()
                  * camera.getHorizontalFOV();
          fieldLocalizer.addVisionMeasurement(
              camera.getSecondaryPose(),
              camera.getFrameTimestamp(),
              VecBuilder.fill(xyStddevSecondary, xyStddevSecondary, Double.POSITIVE_INFINITY));
        }
      }
    }

    closestReefTag = getMinDistanceReefTag();

    for (Camera camera : cameras) {
      if (camera.getCameraDuties().contains(CameraDuty.REEF_LOCALIZATION)
          && !GeometryUtil.isZero(camera.getPrimaryPose())) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        reefLocalizer.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
      }
    }

    setpoint = Reef.reefMap.get(getClosestReefTag()).getPost(getCurrentReefPost());
    distanceToPost =
        RobotState.getRobotPoseReef().getTranslation().getDistance(setpoint.getTranslation());
    atThreshold =
        Math.abs(distanceToPost)
            <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get();

    Logger.recordOutput(
        "RobotState/Pose Data/Estimated Field Pose", fieldLocalizer.getEstimatedPosition());
    Logger.recordOutput(
        "RobotState/Pose Data/Estimated Reef Pose", reefLocalizer.getEstimatedPosition());
    Logger.recordOutput("RobotState/Pose Data/Odometry Pose", odometry.getPoseMeters());
    Logger.recordOutput("RobotState/Pose Data/Heading Offset", headingOffset);
    Logger.recordOutput("RobotState/Pose Data/Closest Reef Tag", closestReefTag);
    Logger.recordOutput("RobotState/Pose Data/Reef Setpoint", setpoint);
    Logger.recordOutput("RobotState/Pose Data/Distance to Post", distanceToPost);
    Logger.recordOutput("RobotState/Pose Data/At Threshold", atThreshold);
  }

  public static Pose2d getRobotPoseField() {
    return fieldLocalizer.getEstimatedPosition();
  }

  public static Pose2d getRobotPoseReef() {
    return reefLocalizer.getEstimatedPosition();
  }

  public static Pose2d getRobotPoseOdometry() {
    return odometry.getPoseMeters();
  }

  private static int getMinDistanceReefTag() {
    int minDistanceTag = 1;
    double minDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 6; i++) {
      double currentDistance =
          RobotState.getRobotPoseReef()
              .getTranslation()
              .getDistance(
                  AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]).getTranslation());
      if (currentDistance < minDistance) {
        minDistance = currentDistance;
        minDistanceTag = i;
      }
    }

    if (AllianceFlipUtil.shouldFlip()) {
      switch (minDistanceTag) {
        case 0:
          minDistanceTag = 7;
          break;
        case 1:
          minDistanceTag = 6;
          break;
        case 2:
          minDistanceTag = 11;
          break;
        case 3:
          minDistanceTag = 10;
          break;
        case 4:
          minDistanceTag = 9;
          break;
        case 5:
          minDistanceTag = 8;
          break;
      }
    } else {
      switch (minDistanceTag) {
        case 0:
          minDistanceTag = 18;
          break;
        case 1:
          minDistanceTag = 19;
          break;
        case 2:
          minDistanceTag = 20;
          break;
        case 3:
          minDistanceTag = 21;
          break;
        case 4:
          minDistanceTag = 22;
          break;
        case 5:
          minDistanceTag = 17;
          break;
      }
    }
    return minDistanceTag;
  }

  public static void resetRobotPose(Pose2d pose) {
    headingOffset = robotHeading.minus(pose.getRotation());
    fieldLocalizer.resetPosition(robotHeading, modulePositions, pose);
    reefLocalizer.resetPosition(robotHeading, modulePositions, pose);
    odometry.resetPosition(robotHeading, modulePositions, pose);
  }
}
