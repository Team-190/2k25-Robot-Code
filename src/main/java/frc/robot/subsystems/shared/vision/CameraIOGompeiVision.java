package frc.robot.subsystems.shared.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shared.vision.VisionConstants.GompeiVisionConfig;
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
  public void updateInputs(CameraIOInputs inputs) {}

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
