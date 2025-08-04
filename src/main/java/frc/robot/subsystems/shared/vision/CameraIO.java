package frc.robot.subsystems.shared.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    public double currentHeartbeat = -1;
    public boolean isConnected = false;
    public Rotation2d[] xOffset = {};
    public Rotation2d[] yOffset = {};
    public boolean[] targetAquired = {};
    public int[] totalTargets = {};
    public double[] averageDistance = {};
    public double[] frameTimestamp = {};
    public Pose2d[] primaryPose = {};
    public Pose2d[] secondaryPose = {};
    public double[] tagIDOfInterest = {};
    public boolean[] useVisionRotation = {};
  }

  public default void updateInputs(CameraIOInputs inputs) {}

  public default boolean getIsConnected(CameraIOInputs inputs) {
    return false;
  }

  public default Rotation2d getXOffset() {
    return new Rotation2d();
  }

  public default Rotation2d getYOffset() {
    return new Rotation2d();
  }

  public default boolean getTargetAquired() {
    return false;
  }

  public default int getTotalTargets() {
    return 0;
  }

  public default double getAverageDistance() {
    return 0.0;
  }

  public default double getFrameTimestamp() {
    return 0.0;
  }

  public default Pose2d getPrimaryPose() {
    return new Pose2d();
  }

  public default Pose2d getSecondaryPose() {
    return new Pose2d();
  }

  public default Pose3d getPoseOfInterest() {
    return new Pose3d();
  }

  public default double getTagIDOfInterest() {
    return -1;
  }

  public default long getPipeline() {
    return 0;
  }

  public default String getName() {
    return "";
  }

  public default CameraType getCameraType() {
    return CameraType.DEFAULT;
  }

  public default double getHorizontalFOV() {
    return 0.0;
  }

  public default double getVerticalFOV() {
    return 0.0;
  }

  public default double getPrimaryXYStandardDeviationCoefficient() {
    return 0.0;
  }

  public default double getThetaStandardDeviationCoefficient() {
    return 0.0;
  }

  public default double getSecondaryXYStandardDeviationCoefficient() {
    return 0.0;
  }

  public default void setPipeline(int pipeline) {}

  public default void setValidTags(int... validIds) {}

  public default void setCameraOffset(Transform3d cameraOffset) {}
}
