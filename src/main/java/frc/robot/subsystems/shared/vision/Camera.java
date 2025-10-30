package frc.robot.subsystems.shared.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  @Getter private final String name;

  @Getter int[] validIds;
  private final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  @Getter Pose2d robotPose;

  public Camera(CameraIO io) {
    inputs = new CameraIOInputsAutoLogged();
    this.io = io;
    this.name = io.getName();

    validIds = FieldConstants.validTags;

    robotPose = Pose2d.kZero;
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

      thetaStdev = Double.POSITIVE_INFINITY;

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
      // ONLY do this if using GompeiVision
      // Loop over each processed frame
      // Only do this if we are running GompeiVision
      if (!(io instanceof CameraIOGompeiVision)) continue;

      for (int i = 0; i < frame.preciseTagIds().length; i++) {
        int tagId = frame.preciseTagIds()[i];

        // Skip invalid tags
        if (java.util.Arrays.stream(validIds).noneMatch(id -> id == tagId)) continue;

        // Get odometry-based pose at the timestamp
        var sample = RobotState.getBufferedPose(frame.timestamp());
        if (sample.isEmpty()) continue;

        // Average tx and ty over four corners
        double tx = 0.0;
        double ty = 0.0;
        for (int j = 0; j < 4; j++) {
          tx += frame.preciseTx()[i][j];
          ty += frame.preciseTy()[i][j];
        }
        tx /= 4.0;
        ty /= 4.0;

        Pose3d cameraPose = io.getGompeiVisionConfig().robotRelativePose();

        // Project 3D distance onto horizontal plane
        double distance2d =
            frame.preciseDistance()[i]
                * Math.cos(-cameraPose.getRotation().getY() - ty); // pitch + tag vertical angle

        // Compute rotation from camera to tag
        Rotation2d camToTagRotation =
            sample
                .get()
                .getRotation()
                .plus(cameraPose.toPose2d().getRotation().plus(Rotation2d.fromRadians(-tx)));

        Pose2d tagPose2d = tagPoses2d.get(tagId);
        if (tagPose2d == null) continue;

        // Compute camera position in field frame
        Translation2d fieldToCameraTranslation =
            new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(GeometryUtil.toTransform2d(distance2d, 0.0))
                .getTranslation();

        // Compute robot pose
        Pose2d robotPose =
            new Pose2d(
                    fieldToCameraTranslation,
                    sample.get().getRotation().plus(cameraPose.toPose2d().getRotation()))
                .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));

        // Use odometry rotation only
        robotPose = new Pose2d(robotPose.getTranslation(), sample.get().getRotation());

        // Store observation for fusion
        xyStdDev =
            io.getPrimaryXYStandardDeviationCoefficient()
                * Math.pow(frame.averageDistance(), 1.2)
                / Math.pow(frame.totalTargets(), 2.0);
        txTyObservations.add(
            new VisionObservation(
                robotPose,
                frame.timestamp(),
                VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY)));
      }
    }

    // Add observations to robot state
    poseObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState::addFieldLocalizerVisionMeasurement);
    // poseObservations.stream()
    //     .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
    //     .forEach(RobotState::addReefLocalizerVisionMeasurement);
    txTyObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState::addReefLocalizerVisionMeasurement);

    List<Pose2d> imprecisePoses = new ArrayList<>();
    for (VisionObservation obs : poseObservations) {
      imprecisePoses.add(obs.pose());
    }

    List<Pose2d> precisePoses = new ArrayList<>();
    for (VisionObservation obs : txTyObservations) {
      precisePoses.add(obs.pose());
    }

    Logger.recordOutput(
        "Vision/Camera/" + name + "/Imprecise Poses", imprecisePoses.toArray(Pose2d[]::new));
    Logger.recordOutput(
        "Vision/Camera/" + name + "/Precise Poses", precisePoses.toArray(Pose2d[]::new));
  }

  public void setValidTags(int... validIds) {
    this.validIds = validIds;
    io.setValidTags(validIds);
  }

  public Pose3d getTransform() {
    return io.getGompeiVisionConfig().robotRelativePose();
  }
}
