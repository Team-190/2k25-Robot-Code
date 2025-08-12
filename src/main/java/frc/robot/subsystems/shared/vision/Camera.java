package frc.robot.subsystems.shared.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystems.shared.vision.CameraIO.ProcessedFrame;
import frc.robot.util.GeometryUtil;
import frc.robot.util.InternalLoggedTracer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Camera {
  private final CameraIOInputsAutoLogged inputs;

  private final CameraIO io;
  private final String name;

  @Getter int[] validIds;
  private final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  public Camera(CameraIO io) {
    inputs = new CameraIOInputsAutoLogged();
    this.io = io;
    this.name = io.getName();

    validIds = FieldConstants.validTags;
  }

  public void periodic() {
    if (tagPoses2d.isEmpty() && io.getFieldLayoutSupplier().get() != null) {
      for (var tag : io.getFieldLayoutSupplier().get().getTags())
        tagPoses2d.put(tag.ID, tag.pose.toPose2d());
    }
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Vision/Cameras/" + name + "Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Vision/Cameras/" + name, inputs);
    InternalLoggedTracer.record("Process Inputs", "Vision/Cameras/" + name + "Periodic");

    List<VisionObservation> poseObservations = new ArrayList<>();
    List<VisionObservation> txTyObservations = new ArrayList<>();

    // Send vision data to robot state
    for (ProcessedFrame frame : inputs.processedFrames) {
      double xyStdevCoeff;
      double thetaStdev;

      if (frame.impreciseIsMultiTag()) {
        xyStdevCoeff = io.getPrimaryXYStandardDeviationCoefficient();
        thetaStdev =
            io.getThetaStandardDeviationCoefficient()
                * Math.pow(frame.averageDistance(), 1.2)
                / Math.pow(frame.totalTargets(), 2.0);
      } else {
        xyStdevCoeff = io.getSecondaryXYStandardDeviationCoefficient();
        thetaStdev = Double.POSITIVE_INFINITY;
      }

      // Add observation to list
      double xyStdDev =
          xyStdevCoeff
              * Math.pow(frame.averageDistance(), 1.2)
              / Math.pow(frame.totalTargets(), 2.0);

      poseObservations.add(
          new VisionObservation(
              frame.imprecisePose(),
              frame.timestamp(),
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));

      // ONLY DO THIS IF WE ARE RUNNING GOMPEIVISION
      if (io instanceof CameraIOGompeiVision) {
        for (int i = 0; i < frame.preciseTagIds().length; i++) {
          final int tagIdIndex = i;
          // Check if tag is valid
          if (java.util.Arrays.stream(validIds)
              .noneMatch(id -> id == frame.preciseTagIds()[tagIdIndex])) {
            continue;
          }
          // Get rotation at timestamp
          var sample = RobotState.getBufferedPose(frame.timestamp());
          if (sample.isEmpty()) {
            // exit if not there
            return;
          }
          Rotation2d robotRotation =
              RobotState.getRobotPoseField()
                  .transformBy(new Transform2d(RobotState.getRobotPoseOdometry(), sample.get()))
                  .getRotation();

          // Average tx's and ty's
          double tx = 0.0;
          double ty = 0.0;
          for (int j = 0; j < 4; j++) {
            tx += frame.preciseTx()[i][j];
            ty += frame.preciseTy()[i][j];
          }
          tx /= 4.0;
          ty /= 4.0;

          Pose3d cameraPose =
              new Pose3d(
                  io.getGompeiVisionConfig().robotToCameraTransform().getTranslation(),
                  io.getGompeiVisionConfig().robotToCameraTransform().getRotation());

          // Use 3D distance and tag angles to find robot pose
          Translation2d camToTagTranslation =
              new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
                  .transformBy(
                      new Transform3d(
                          new Translation3d(frame.preciseDistance()[i], 0, 0), Rotation3d.kZero))
                  .getTranslation()
                  .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
                  .toTranslation2d();
          Rotation2d camToTagRotation =
              robotRotation.plus(
                  cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
          var tagPose2d = tagPoses2d.get(frame.preciseTagIds()[i]);
          if (tagPose2d == null) return;
          Translation2d fieldToCameraTranslation =
              new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                  .transformBy(GeometryUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
                  .getTranslation();
          Pose2d robotPose =
              new Pose2d(
                      fieldToCameraTranslation,
                      robotRotation.plus(cameraPose.toPose2d().getRotation()))
                  .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
          // Use gyro angle at time for robot rotation
          robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

          // Add transform to current odometry based pose for latency correction
          txTyObservations.add(
              new VisionObservation(
                  robotPose,
                  frame.timestamp(),
                  VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY)));
        }
      }
    }

    // Add observations to robot state
    poseObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState::addFieldLocalizerVisionMeasurement);
    poseObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState::addReefLocalizerVisionMeasurement);
    // txTyObservations.stream()
    //     .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
    //     .forEach(RobotState::addReefLocalizerVisionMeasurement);
  }

  public void setValidTags(int... validIds) {
    this.validIds = validIds;
    io.setValidTags(validIds);
  }
}
