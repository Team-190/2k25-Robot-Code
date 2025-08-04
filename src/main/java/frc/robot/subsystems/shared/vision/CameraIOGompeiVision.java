package frc.robot.subsystems.shared.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.vision.VisionConstants.GompeiVisionConfig;
import frc.robot.util.InternalLoggedTracer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import lombok.Getter;

public class CameraIOGompeiVision implements CameraIO {
  private final GompeiVisionConfig config;

  private final NetworkTable configTable;
  private final NetworkTable outputTable;

  @Getter private final String name;
  private final String deviceID;
  // private final CameraType cameraType;

  private final DoubleArraySubscriber observationSubscriber;
  // private final IntegerSubscriber fpsAprilTagSubscriber;

  public CameraIOGompeiVision(GompeiVisionConfig config) {
    this.config = config;

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
    InternalLoggedTracer.reset();

    inputs.isConnected = getIsConnected(inputs);
    if (inputs.isConnected) {
      TimestampedDoubleArray[] aprilTagQueue = observationSubscriber.readQueue();
      double[] timestamps = new double[aprilTagQueue.length];
      double[][] frames = new double[aprilTagQueue.length][];
      for (int i = 0; i < aprilTagQueue.length; i++) {
        timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
        frames[i] = aprilTagQueue[i].value;
      }
      inputs.frameTimestamp = timestamps;
      // TimestampedInteger[] fps = fpsAprilTagSubscriber.readQueue();

      // List<Rotation2d> xOffsets = new ArrayList<>();
      // List<Rotation2d> yOffsets = new ArrayList<>();
      // List<Boolean> targetAquired = new ArrayList<>();
      inputs.totalTargets = new int[frames.length];
      inputs.averageDistance = new double[frames.length];
      inputs.primaryPose = new Pose2d[frames.length];
      inputs.useVisionRotation = new boolean[frames.length];

      for (int frameIndex = 0; frameIndex < frames.length; frameIndex++) {
        Pose3d cameraPose = new Pose3d();
        Pose2d robotPose = new Pose2d();
        boolean isMultiTag = false;

        double[] values = frames[frameIndex];
        switch ((int) values[0]) {
            // Multitag
          case 1:
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            isMultiTag = true;
            robotPose =
                cameraPose.transformBy(this.config.robotToCameraTransform().inverse()).toPose2d();
            break;
            // Singletag (disambiguate)
          case 2:
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

            Pose2d robotPose0 =
                cameraPose0.transformBy(this.config.robotToCameraTransform().inverse()).toPose2d();
            Pose2d robotPose1 =
                cameraPose1.transformBy(this.config.robotToCameraTransform().inverse()).toPose2d();

            // Check for ambiguity and select based on estimated rotation
            if (error0 < error1 * VisionConstants.AMBIGUITY_THRESHOLD
                || error1 < error0 * VisionConstants.AMBIGUITY_THRESHOLD) {
              Rotation2d currentRotation = RobotState.getRobotPoseField().getRotation();
              Rotation2d visionRotation0 = robotPose0.getRotation();
              Rotation2d visionRotation1 = robotPose1.getRotation();
              if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                  < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                cameraPose = cameraPose0;
                robotPose = robotPose0;
              } else {
                cameraPose = cameraPose1;
                robotPose = robotPose1;
              }
            }
            break;
        }

        // Exit if no data
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
          Optional<Pose3d> tagPose =
              AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
                  .getTagPose(tagId);
          tagPose.ifPresent(tagPoses::add);
        }
        if (tagPoses.isEmpty()) continue;

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        inputs.totalTargets[frameIndex] = tagPoses.size();
        inputs.averageDistance[frameIndex] = avgDistance;
        inputs.primaryPose[frameIndex] = robotPose;
        inputs.useVisionRotation[frameIndex] = isMultiTag;
      }
    }

    InternalLoggedTracer.record("Update Inputs", "Vision/Cameras/" + name + "/GompeiVision");
  }

  @Override
  public boolean getIsConnected(CameraIOInputs inputs) {
    return true;
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
  public String toString() {
    return name;
  }
}
