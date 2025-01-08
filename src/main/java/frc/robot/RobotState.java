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
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.util.GeometryUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  @Getter private static ControlData controlData = new ControlData();

  @Getter @Setter private static double speakerFlywheelCompensation = 0.0;
  @Getter @Setter private static double speakerAngleCompensation = 0.0;

  private static final SwerveDrivePoseEstimator poseEstimator;
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

    poseEstimator =
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
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), robotHeading, modulePositions);

    for (Camera camera : cameras) {
      double[] limelightHeadingData = {
        robotHeading.minus(headingOffset).getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
      };
      camera.getRobotHeadingPublisher().set(limelightHeadingData, latestRobotHeadingTimestamp);
    }
    NetworkTableInstance.getDefault().flush();
    for (Camera camera : cameras) {

      if (camera.getTargetAquired()
          && !GeometryUtil.isZero(camera.getPrimaryPose())
          && !GeometryUtil.isZero(camera.getSecondaryPose())
          && Math.abs(robotYawVelocity) <= Units.degreesToRadians(15.0)
          && Math.abs(robotFieldRelativeVelocity.getNorm()) <= 1.0) {
        double xyStddevPrimary =
            camera.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(camera.getAverageDistance(), 2.0)
                / camera.getTotalTargets()
                * camera.getHorizontalFOV();
        poseEstimator.addVisionMeasurement(
            camera.getPrimaryPose(),
            camera.getFrameTimestamp(),
            VecBuilder.fill(xyStddevPrimary, xyStddevPrimary, Double.POSITIVE_INFINITY));
        if (camera.getTotalTargets() > 1) {
          double xyStddevSecondary =
              camera.getSecondaryXYStandardDeviationCoefficient()
                  * Math.pow(camera.getAverageDistance(), 2.0)
                  / camera.getTotalTargets()
                  * camera.getHorizontalFOV();
          poseEstimator.addVisionMeasurement(
              camera.getSecondaryPose(),
              camera.getFrameTimestamp(),
              VecBuilder.fill(xyStddevSecondary, xyStddevSecondary, Double.POSITIVE_INFINITY));
        }
      }
    }

    controlData = new ControlData();

    Logger.recordOutput(
        "RobotState/Pose Data/Estimated Pose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput("RobotState/Pose Data/Odometry Pose", odometry.getPoseMeters());
    Logger.recordOutput("RobotState/Pose Data/Heading Offset", headingOffset);
  }

  public static Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public static void resetRobotPose(Pose2d pose) {
    headingOffset = robotHeading.minus(pose.getRotation());
    poseEstimator.resetPosition(robotHeading, modulePositions, pose);
    odometry.resetPosition(robotHeading, modulePositions, pose);
  }

  public static record ControlData() {}
}
