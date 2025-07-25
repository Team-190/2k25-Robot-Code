package frc.robot.subsystems.shared.visionlimelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import frc.robot.util.InternalLoggedTracer;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Camera {
  private final CameraIOInputsAutoLogged inputs;

  private final CameraIO io;
  @Getter private final String name;
  @Getter private final CameraType cameraType;
  @Getter private final double horizontalFOV;
  @Getter private final double verticalFOV;
  @Getter private final double primaryXYStandardDeviationCoefficient;
  @Getter private final double secondaryXYStandardDeviationCoefficient;
  @Getter private final DoubleArrayPublisher robotHeadingPublisher;
  @Getter private final List<CameraDuty> cameraDuties;

  public Camera(
      CameraIO io,
      double horizontalFOV,
      double verticalFOV,
      double primaryXYStandardDeviationCoefficient,
      double secondaryXYStandardDeviationCoefficient,
      DoubleArrayPublisher robotHeadingPublisher,
      List<CameraDuty> cameraDuties,
      Transform3d offset) {
    inputs = new CameraIOInputsAutoLogged();
    this.io = io;
    io.setCameraOffset(offset);
    this.name = io.getName();
    this.cameraType = io.getCameraType();
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.primaryXYStandardDeviationCoefficient = primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient = secondaryXYStandardDeviationCoefficient;
    this.robotHeadingPublisher = robotHeadingPublisher;
    this.cameraDuties = cameraDuties;
  }

  public Camera(
      CameraIO io,
      String name,
      double horizontalFOV,
      double verticalFOV,
      double primaryXYStandardDeviationCoefficient,
      double secondaryXYStandardDeviationCoefficient,
      DoubleArrayPublisher robotHeadingPublisher,
      List<CameraDuty> cameraDuties) {
    inputs = new CameraIOInputsAutoLogged();
    this.io = io;
    this.name = name;
    this.cameraType = io.getCameraType();
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.primaryXYStandardDeviationCoefficient = primaryXYStandardDeviationCoefficient;
    this.secondaryXYStandardDeviationCoefficient = secondaryXYStandardDeviationCoefficient;
    this.robotHeadingPublisher = robotHeadingPublisher;
    this.cameraDuties = cameraDuties;
  }

  public void periodic() {
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Vision/Cameras/" + name + "Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Vision/Cameras/" + name, inputs);
    InternalLoggedTracer.record("Process Inputs", "Vision/Cameras/" + name + "Periodic");
  }

  public Rotation2d getXOffset() {
    return inputs.xOffset;
  }

  public Rotation2d getYOffset() {
    return inputs.yOffset;
  }

  public boolean getTargetAquired() {
    return inputs.targetAquired;
  }

  public int getTotalTargets() {
    return inputs.totalTargets;
  }

  public double getAverageDistance() {
    return inputs.averageDistance;
  }

  public double getFrameTimestamp() {
    return inputs.frameTimestamp;
  }

  public Pose2d getPrimaryPose() {
    return inputs.primaryPose;
  }

  public Pose2d getSecondaryPose() {
    return inputs.secondaryPose;
  }

  public double getTagIDOfInterest() {
    return inputs.tagIDOfInterest;
  }

  public boolean getIsConnected() {
    return inputs.isConnected;
  }

  public void setPipeline(int pipeline) {
    io.setPipeline(pipeline);
  }

  public void setValidTags(int... validIds) {
    io.setValidTags(validIds);
  }
}
