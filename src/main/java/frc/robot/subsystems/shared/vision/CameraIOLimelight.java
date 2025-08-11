package frc.robot.subsystems.shared.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.vision.VisionConstants.LimelightConfig;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.LimelightHelpers;
import lombok.Getter;

public class CameraIOLimelight implements CameraIO {
  private final LimelightConfig config;

  @Getter private final String name;
  @Getter private final DoubleArrayPublisher headingPublisher;

  public CameraIOLimelight(LimelightConfig config) {
    this.config = config;
    this.name = "limelight-" + this.config.key();
    this.headingPublisher =
        NetworkTableInstance.getDefault()
            .getTable(this.name)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish();
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    InternalLoggedTracer.reset();
    inputs.currentHeartbeat =
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("hb").getDouble(-1);
    inputs.isConnected = getIsConnected(inputs);

    if (inputs.isConnected) {
      this.headingPublisher.set(
          new double[] {
            RobotState.getHeadingData().robotHeading().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
          },
          RobotState.getHeadingData().latestRobotHeadingTimestamp());

      inputs.processedFrames =
          new ProcessedFrame[] {
            new ProcessedFrame(
                LimelightHelpers.getBotPoseEstimate_wpiBlue(name).timestampSeconds,
                LimelightHelpers.getTargetCount(name),
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).avgTagDist,
                new ProcessedPreciseLocalPositioningData(
                    new int[] {(int) LimelightHelpers.getFiducialID(name)},
                    new double[][] {{}},
                    new double[][] {{}},
                    new double[] {
                      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).avgTagDist
                    }),
                new ProcessedImpreciseGlobalPositioningData(
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose,
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).tagCount > 1))
          };
    }
    InternalLoggedTracer.record("Update Inputs", "Vision/Cameras/" + name + "/Limelight");
  }

  @Override
  public boolean getIsConnected(CameraIOInputs inputs) {
    return inputs.currentHeartbeat != -1
        && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) != null
        && LimelightHelpers.getBotPoseEstimate_wpiBlue(name) != null;
  }

  @Override
  public String toString() {
    return name;
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
  public void setCameraOffset(Transform3d cameraOffset) {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        cameraOffset.getX(),
        cameraOffset.getY(),
        cameraOffset.getZ(),
        Units.radiansToDegrees(cameraOffset.getRotation().getX()),
        Units.radiansToDegrees(cameraOffset.getRotation().getY()),
        Units.radiansToDegrees(cameraOffset.getRotation().getZ()));
  }
}
