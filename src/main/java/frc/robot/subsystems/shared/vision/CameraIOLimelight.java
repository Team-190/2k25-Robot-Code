package frc.robot.subsystems.shared.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LimelightHelpers;
import lombok.Getter;

public class CameraIOLimelight implements CameraIO {
  private final String name;
  @Getter private final CameraType cameraType;
  @Getter private final double horizontalFOV;
  @Getter private final double verticalFOV;
  @Getter private final double primaryXYStandardDeviationCoefficient;
  @Getter private final double secondaryXYStandardDeviationCoefficient;

  public CameraIOLimelight(String name, CameraType cameraType) {
    this.name = "limelight-" + name;
    this.cameraType = cameraType;
    this.horizontalFOV = cameraType.horizontalFOV;
    this.verticalFOV = cameraType.verticalFOV;
    this.primaryXYStandardDeviationCoefficient = cameraType.primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient =
        cameraType.secondaryXYStandardDeviationCoefficient;
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    inputs.xOffset = Rotation2d.fromDegrees(LimelightHelpers.getTX(name));
    inputs.yOffset = Rotation2d.fromDegrees(LimelightHelpers.getTY(name));
    inputs.targetAquired = LimelightHelpers.getTV(name);
    inputs.totalTargets = LimelightHelpers.getTargetCount(name);
    inputs.averageDistance = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).avgTagDist;
    inputs.primaryPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose;
    inputs.secondaryPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(name).pose;
    inputs.frameTimestamp = LimelightHelpers.getBotPoseEstimate_wpiBlue(name).timestampSeconds;
    inputs.poseOfInterest = LimelightHelpers.getTargetPose3d_RobotSpace(name);
    inputs.tagIDOfInterest = LimelightHelpers.getFiducialID(name);
  }

  @Override
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  @Override
  public void setValidTags(int... validIds) {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, validIds);
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public String toString() {
    return name;
  }

  @Override
  public void setCameraPose(Pose3d robotRelativePose) {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        robotRelativePose.getX(),
        -robotRelativePose.getY(),
        robotRelativePose.getZ(),
        Units.radiansToDegrees(robotRelativePose.getRotation().getX()),
        Units.radiansToDegrees(robotRelativePose.getRotation().getY()),
        Units.radiansToDegrees(robotRelativePose.getRotation().getZ()));
  }
}
