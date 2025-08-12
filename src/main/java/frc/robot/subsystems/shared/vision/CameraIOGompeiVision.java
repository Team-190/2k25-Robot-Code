package frc.robot.subsystems.shared.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.vision.VisionConstants.GompeiVisionConfig;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CameraIOGompeiVision implements CameraIO {
  private final GompeiVisionConfig config;
  private final Supplier<AprilTagFieldLayout> aprilTagLayoutSupplier;

  private final NetworkTable configTable;
  private final NetworkTable outputTable;

  @Getter private final String name;
  private final String deviceID;
  // private final CameraType cameraType;

  private final DoubleArraySubscriber observationSubscriber;
  // private final IntegerSubscriber fpsAprilTagSubscriber;

  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  public CameraIOGompeiVision(
      GompeiVisionConfig config, Supplier<AprilTagFieldLayout> aprilTagLayoutSupplier) {
    this.config = config;
    this.aprilTagLayoutSupplier = aprilTagLayoutSupplier;

    this.name = this.config.key();

    this.outputTable =
        NetworkTableInstance.getDefault()
            .getTable("cameras")
            .getSubTable(this.name)
            .getSubTable("output");
    this.configTable =
        NetworkTableInstance.getDefault()
            .getTable("cameras")
            .getSubTable(this.name)
            .getSubTable("config");

    this.deviceID = this.config.hardwareID();
    // this.cameraType = this.config.cameraType();

    this.configTable.getStringTopic("role").publish().set(name);
    this.configTable.getStringTopic("hardware_id").publish().set(deviceID);
    this.configTable
        .getDoubleArrayTopic("camera_matrix")
        .publish()
        .set(this.config.cameraMatrix().getData());
    this.configTable
        .getDoubleArrayTopic("distortion_coefficients")
        .publish()
        .set(this.config.distortionCoefficients().getData());
    this.configTable.getDoubleTopic("exposure").publish().set(this.config.exposure());
    this.configTable.getDoubleTopic("gain").publish().set(this.config.gain());
    this.configTable.getIntegerTopic("width").publish().set(this.config.width());
    this.configTable.getIntegerTopic("height").publish().set(this.config.height());
    this.configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
    this.configTable.getBooleanTopic("setup_mode").publish().set(false);

    this.observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    // this.fpsAprilTagSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    double lastFrameTime = 0;

    // Get AprilTag data
    var aprilTagQueue = observationSubscriber.readQueue();
    var timestamps = new double[aprilTagQueue.length];
    var frames = new double[aprilTagQueue.length][];

    for (int i = 0; i < aprilTagQueue.length; i++) {
      timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
      frames[i] = aprilTagQueue[i].value;
    }

    List<ProcessedFrame> processedFrames = new ArrayList<>();

    for (int frameIndex = 0; frameIndex < aprilTagQueue.length; frameIndex++) {
      lastFrameTime = Timer.getTimestamp();
      double timestamp = timestamps[frameIndex];
      int totalTargets = 0;
      double averageDistance = 0.0;
      double[] values = frames[frameIndex];

      if (values.length == 0 || values[0] == 0) {
        continue;
      }

      Pose3d cameraPose = null;
      Pose2d robotPose = null;

      switch ((int) values[0]) {
        case 1:
          // One pose
          cameraPose =
              new Pose3d(
                  values[2],
                  values[3],
                  values[4],
                  new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
          robotPose = cameraPose.transformBy(config.robotToCameraTransform().inverse()).toPose2d();
          break;

        case 2:
          // Multiple poses
          double error0 = values[1];
          double error1 = values[9];
          Pose3d cameraPose0 =
              new Pose3d(
                  values[2],
                  values[3],
                  values[4],
                  new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));

          Pose3d cameraPose1 =
              new Pose3d(
                  values[10],
                  values[11],
                  values[12],
                  new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
          Transform3d cameraToRobot = config.robotToCameraTransform().inverse();

          Pose2d robotPose0 = cameraPose0.transformBy(cameraToRobot).toPose2d();
          Pose2d robotPose1 = cameraPose1.transformBy(cameraToRobot).toPose2d();

          if (error0 < error1 * VisionConstants.AMBIGUITY_THRESHOLD
              || error1 < error0 * VisionConstants.AMBIGUITY_THRESHOLD) {
            Rotation2d currentRotation = RobotState.getHeadingData().robotHeading();
            Rotation2d visionRotation0 = robotPose0.getRotation();
            Rotation2d visionRotation1 = robotPose1.getRotation();
            if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
              robotPose = robotPose0;
              cameraPose = cameraPose0;
            } else {
              robotPose = robotPose1;
              cameraPose = cameraPose1;
            }
          }

          break;
        default:
          // Invalid frame
          continue;
      }

      if (cameraPose == null || robotPose == null) {
        continue;
      }

      // Exit if robot pose is off the field
      if (robotPose.getX() < -VisionConstants.FIELD_BORDER_MARGIN
          || robotPose.getX() > FieldConstants.fieldLength + VisionConstants.FIELD_BORDER_MARGIN
          || robotPose.getY() < -VisionConstants.FIELD_BORDER_MARGIN
          || robotPose.getY() > FieldConstants.fieldWidth + VisionConstants.FIELD_BORDER_MARGIN) {
        continue;
      }

      // Get tag poses and update last detection times
      List<Pose3d> tagPoses = new ArrayList<>();
      for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i += 10) {
        int tagId = (int) values[i];
        lastTagDetectionTimes.put(tagId, Timer.getTimestamp());
        Optional<Pose3d> tagPose = aprilTagLayoutSupplier.get().getTagPose((int) values[i]);
        tagPose.ifPresent(tagPoses::add);
      }

      int[] tagIds = new int[tagPoses.size()];
      double[][] txs = new double[tagPoses.size()][];
      double[][] tys = new double[tagPoses.size()][];
      double[] distances = new double[tagPoses.size()];

      if (!tagPoses.isEmpty()) {
        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        averageDistance = totalDistance / tagPoses.size();
        totalTargets = tagPoses.size();

        // Add TxTy observations
        int tagEstimationDataEndIndex =
            switch ((int) values[0]) {
              default -> 0;
              case 1 -> 8;
              case 2 -> 16;
            };

        int indexCounter = 0;
        for (int index = tagEstimationDataEndIndex + 1; index < values.length; index += 10) {
          double[] tx = new double[4];
          double[] ty = new double[4];
          for (int i = 0; i < 4; i++) {
            tx[i] = values[index + 1 + (2 * i)];
            ty[i] = values[index + 1 + (2 * i) + 1];
          }
          int tagId = (int) values[index];
          double distance = values[index + 9];

          tagIds[indexCounter] = tagId;
          txs[indexCounter] = tx;
          tys[indexCounter] = ty;
          distances[indexCounter] = distance;
          indexCounter++;
        }
      }

      // Update inputs
      inputs.currentHeartbeat = Timer.getFPGATimestamp();
      inputs.isConnected = true;
      processedFrames.add(
          new ProcessedFrame(
              timestamp,
              totalTargets,
              averageDistance,
              tagIds,
              txs,
              tys,
              distances,
              robotPose,
              totalTargets > 1));

      Logger.recordOutput(
          "AprilTagVision/Inst" + config.key() + "/LatencySecs", Timer.getTimestamp() - timestamp);
      Logger.recordOutput("AprilTagVision/Inst" + config.key() + "/RobotPose", robotPose);
      Logger.recordOutput(
          "AprilTagVision/Inst" + config.key() + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
    }

    inputs.processedFrames = processedFrames.toArray(new ProcessedFrame[processedFrames.size()]);

    // If no frames from instances, clear robot pose
    if (aprilTagQueue.length == 0) {
      Logger.recordOutput("AprilTagVision/Inst" + config.key() + "/RobotPose", Pose2d.kZero);
    }

    // If no recent frames from instance, clear tag poses
    if (Timer.getTimestamp() - lastFrameTime > VisionConstants.TARGET_LOG_TIME_SECS) {
      Logger.recordOutput("AprilTagVision/Inst" + config.key() + "/TagPoses", new Pose3d[] {});
    }

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getTimestamp() - detectionEntry.getValue() < VisionConstants.TARGET_LOG_TIME_SECS) {
        aprilTagLayoutSupplier
            .get()
            .getTagPose(detectionEntry.getKey())
            .ifPresent(allTagPoses::add);
      }
    }
    Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));
  }

  @Override
  public boolean getIsConnected(CameraIOInputs inputs) {
    return true;
  }

  @Override
  public GompeiVisionConfig getGompeiVisionConfig() {
    return this.config;
  }

  @Override
  public double getPrimaryXYStandardDeviationCoefficient() {
    return this.config.multitagXYStdev();
  }

  @Override
  public double getThetaStandardDeviationCoefficient() {
    return this.config.thetaStdev();
  }

  @Override
  public double getSecondaryXYStandardDeviationCoefficient() {
    return this.config.singletagXYStdev();
  }

  @Override
  public Supplier<AprilTagFieldLayout> getFieldLayoutSupplier() {
    return aprilTagLayoutSupplier;
  }

  @Override
  public String toString() {
    return name;
  }
}
