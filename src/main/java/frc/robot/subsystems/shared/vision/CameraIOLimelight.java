package frc.robot.subsystems.shared.vision;

import edu.wpi.first.math.geometry.Rotation2d;
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
  }

  @Override
  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public String toString() {
    return name;
  }
}
