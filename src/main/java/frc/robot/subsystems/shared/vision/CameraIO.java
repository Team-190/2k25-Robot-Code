package frc.robot.subsystems.shared.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.shared.vision.VisionConstants.GompeiVisionConfig;
import frc.robot.subsystems.shared.vision.VisionConstants.LimelightConfig;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    double currentHeartbeat = -1;
    boolean isConnected = false;
    ProcessedFrame[] processedFrames = {};
  }

  public default void updateInputs(CameraIOInputs inputs) {}

  public default String getName() {
    return "";
  }

  public default boolean getIsConnected(CameraIOInputs inputs) {
    return false;
  }

  public default LimelightConfig getLimelightConfig() {
    return LimelightConfig.builder().build();
  }

  public default GompeiVisionConfig getGompeiVisionConfig() {
    return GompeiVisionConfig.builder().build();
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

  public default Supplier<AprilTagFieldLayout> getFieldLayoutSupplier() {
    return () -> null;
  }

  public default List<Pose3d> getTagPoses() {
    return List.of();
  }

  public default Pose2d getRobotPose() {
    return new Pose2d();
  }

  public record ProcessedFrame(
      double timestamp,
      int totalTargets,
      double averageDistance,
      int[] preciseTagIds,
      double[][] preciseTx,
      double[][] preciseTy,
      double[] preciseDistance,
      Pose2d imprecisePose,
      boolean impreciseIsMultiTag) {

    @Override
    public String toString() {
      return "ProcessedFrame{"
          + "timestamp="
          + timestamp
          + ", totalTargets="
          + totalTargets
          + ", averageDistance="
          + averageDistance
          + ", preciseTagIds="
          + java.util.Arrays.toString(preciseTagIds)
          + ", preciseTx="
          + java.util.Arrays.deepToString(preciseTx)
          + ", preciseTy="
          + java.util.Arrays.deepToString(preciseTy)
          + ", preciseDistance="
          + java.util.Arrays.toString(preciseDistance)
          + ", imprecisePose="
          + imprecisePose
          + ", impreciseIsMultiTag="
          + impreciseIsMultiTag
          + '}';
    }
  }
}
