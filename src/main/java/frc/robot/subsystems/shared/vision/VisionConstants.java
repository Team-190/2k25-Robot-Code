package frc.robot.subsystems.shared.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shared.vision.CameraConstants.Limelight2PlusConstants;
import frc.robot.subsystems.shared.vision.CameraConstants.Limelight3GConstants;
import frc.robot.subsystems.shared.vision.CameraConstants.Limelight4Constants;
import frc.robot.subsystems.shared.vision.CameraConstants.ThriftyCamConstants;
import java.util.List;
import lombok.Builder;

public class VisionConstants {
  public static final double AMBIGUITY_THRESHOLD = 0.4;
  public static final double FIELD_BORDER_MARGIN = 0.5;
  public static final double TARGET_LOG_TIME_SECS = 0.1;

  public static class RobotCameras {

    private static final LimelightConfig V0_FUNKY_CENTER =
        LimelightConfig.builder()
            .key("center")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(Limelight3GConstants.HORIZONTAL_FOV)
            .verticalFOV(Limelight3GConstants.VERTICAL_FOV)
            .megatagXYStdev(Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight3GConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    0,
                    0.241,
                    0.2,
                    new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(-90))))
            .build();

    private static final LimelightConfig V0_FUNKY_LEFT =
        LimelightConfig.builder()
            .key("left")
            .cameraType(CameraType.LIMELIGHT_2_PLUS)
            .horizontalFOV(Limelight2PlusConstants.HORIZONTAL_FOV)
            .verticalFOV(Limelight2PlusConstants.VERTICAL_FOV)
            .megatagXYStdev(Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight2PlusConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    0.284,
                    0.1884,
                    0.22,
                    new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(-135))))
            .build();

    private static final LimelightConfig V0_FUNKY_RIGHT =
        LimelightConfig.builder()
            .key("right")
            .cameraType(CameraType.LIMELIGHT_2_PLUS)
            .horizontalFOV(Limelight2PlusConstants.HORIZONTAL_FOV)
            .verticalFOV(Limelight2PlusConstants.VERTICAL_FOV)
            .megatagXYStdev(Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight2PlusConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    -0.284,
                    0.1883,
                    0.22,
                    new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(-45))))
            .build();

    private static final LimelightConfig V1_STACKUP_CENTER =
        LimelightConfig.builder()
            .key("center")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(Limelight3GConstants.HORIZONTAL_FOV)
            .verticalFOV(Limelight3GConstants.VERTICAL_FOV)
            .megatagXYStdev(Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight3GConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    -0.211842, 0.0, 0.226176, new Rotation3d(0, 0, Units.degreesToRadians(-180))))
            .build();

    private static final LimelightConfig V1_STACKUP_LEFT =
        LimelightConfig.builder()
            .key("left")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(Limelight3GConstants.HORIZONTAL_FOV)
            .verticalFOV(Limelight3GConstants.VERTICAL_FOV)
            .megatagXYStdev(Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight3GConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    -0.203974,
                    -0.281026,
                    0.237475,
                    new Rotation3d(0.0, 0, Units.degreesToRadians(225))))
            .build();

    private static final LimelightConfig V1_STACKUP_RIGHT =
        LimelightConfig.builder()
            .key("right")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(Limelight3GConstants.HORIZONTAL_FOV)
            .verticalFOV(Limelight3GConstants.VERTICAL_FOV)
            .megatagXYStdev(Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight3GConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    -0.203974,
                    0.281026,
                    0.237475,
                    new Rotation3d(0, 0, Units.degreesToRadians(-225))))
            .build();

    private static final LimelightConfig V2_REDUNDANCY_CENTER =
        LimelightConfig.builder()
            .key("center")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(Limelight3GConstants.HORIZONTAL_FOV)
            .verticalFOV(Limelight3GConstants.VERTICAL_FOV)
            .megatagXYStdev(Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight3GConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    -0.211842,
                    -0.004974,
                    0.221448,
                    new Rotation3d(0, 0, Units.degreesToRadians(-180))))
            .build();

    private static final LimelightConfig V2_REDUNDANCY_LEFT =
        LimelightConfig.builder()
            .key("left")
            .cameraType(CameraType.LIMELIGHT_4)
            .horizontalFOV(Limelight4Constants.HORIZONTAL_FOV)
            .verticalFOV(Limelight4Constants.VERTICAL_FOV)
            .megatagXYStdev(Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight4Constants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    -0.204072,
                    -0.280928,
                    0.231125,
                    new Rotation3d(0.0, 0, Units.degreesToRadians(225))))
            .build();

    private static final LimelightConfig V2_REDUNDANCY_RIGHT =
        LimelightConfig.builder()
            .key("right")
            .cameraType(CameraType.LIMELIGHT_4)
            .horizontalFOV(Limelight4Constants.HORIZONTAL_FOV)
            .verticalFOV(Limelight4Constants.VERTICAL_FOV)
            .megatagXYStdev(Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .metatagThetaStdev(Limelight4Constants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT)
            .megatag2XYStdev(Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    -0.204072,
                    0.280928,
                    0.231125,
                    new Rotation3d(0, 0, Units.degreesToRadians(-225))))
            .build();

    private static final GompeiVisionConfig BACK_BOTTOM_LEFT = // Back Right robot relative
        GompeiVisionConfig.builder()
            .key("camera_backbottomleft")
            .hardwareID("camera_backbottomleft")
            .cameraType(CameraType.THRIFTYCAM)
            .exposure(50.0)
            .gain(0.0)
            .horizontalFOV(ThriftyCamConstants.HORIZONTAL_FOV)
            .width(ThriftyCamConstants.WIDTH)
            .height(ThriftyCamConstants.HEIGHT)
            .cameraMatrix(
                new Matrix<N3, N3>(
                    MatBuilder.fill(
                        N3.instance,
                        N3.instance,
                        1381.859898393546,
                        0,
                        741.0665606797552,
                        0,
                        1388.430990713648,
                        626.5683262179745,
                        0,
                        0,
                        1)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(
                        N5.instance,
                        N1.instance,
                        -0.026870821933410,
                        0.034738098307543,
                        -0.0005183171,
                        0.0005800466,
                        -0.050574364954119)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    Units.inchesToMeters(-11.0),
                    Units.inchesToMeters(-11.75),
                    Units.inchesToMeters(9.182678),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-65))))
            .build();

    private static final GompeiVisionConfig BACK_RIGHT = // Front Left robot relative
        GompeiVisionConfig.builder()
            .key("camera_backright")
            .hardwareID("camera_backright")
            .cameraType(CameraType.THRIFTYCAM)
            .exposure(50.0)
            .gain(0.0)
            .horizontalFOV(ThriftyCamConstants.HORIZONTAL_FOV)
            .width(ThriftyCamConstants.WIDTH)
            .height(ThriftyCamConstants.HEIGHT)
            .cameraMatrix(
                new Matrix<N3, N3>(
                    MatBuilder.fill(
                        N3.instance,
                        N3.instance,
                        1364.233774114576,
                        0,
                        833.9370492297464,
                        0,
                        1359.524805084978,
                        697.4271592073976,
                        0,
                        0,
                        1)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(
                        N5.instance,
                        N1.instance,
                        -0.032643825214208,
                        0.006677356066269,
                        0.0009003298,
                        0.0000022707354437,
                        0.004509255043043)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    Units.inchesToMeters(11.0),
                    Units.inchesToMeters(11.75),
                    Units.inchesToMeters(9.182678),
                    new Rotation3d(
                        0, Units.degreesToRadians(-20.0), Units.degreesToRadians(115.0))))
            .build();

    private static final GompeiVisionConfig FRONT_RIGHT = // Back Left robot relative
        GompeiVisionConfig.builder()
            .key("camera_frontright")
            .hardwareID("camera_frontright")
            .cameraType(CameraType.THRIFTYCAM)
            .exposure(50.0)
            .gain(0.0)
            .horizontalFOV(ThriftyCamConstants.HORIZONTAL_FOV)
            .width(ThriftyCamConstants.WIDTH)
            .height(ThriftyCamConstants.HEIGHT)
            .cameraMatrix(
                new Matrix<N3, N3>(
                    MatBuilder.fill(
                        N3.instance,
                        N3.instance,
                        1369.692474966544,
                        0,
                        787.7364691969475,
                        0,
                        1369.438576249971,
                        623.5492216309891,
                        0,
                        0,
                        1)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(
                        N5.instance,
                        N1.instance,
                        -0.012505958090617,
                        -0.055114601985320,
                        0.000799901335506,
                        0.003926554248417,
                        0.079675565790226)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    Units.inchesToMeters(-11.0),
                    Units.inchesToMeters(11.75),
                    Units.inchesToMeters(9.182678),
                    new Rotation3d(0, Units.degreesToRadians(-20.0), Units.degreesToRadians(65.0))))
            .build();

    private static final GompeiVisionConfig FRONT_LEFT = // Front Right robot relative
        GompeiVisionConfig.builder()
            .key("camera_frontleft")
            .hardwareID("camera_frontleft")
            .cameraType(CameraType.THRIFTYCAM)
            .exposure(50.0)
            .gain(0.0)
            .horizontalFOV(ThriftyCamConstants.HORIZONTAL_FOV)
            .width(ThriftyCamConstants.WIDTH)
            .height(ThriftyCamConstants.HEIGHT)
            .cameraMatrix(
                new Matrix<N3, N3>(
                    MatBuilder.fill(
                        N3.instance,
                        N3.instance,
                        1387.928742097397,
                        0,
                        736.9807673004121,
                        0,
                        1380.979555171942,
                        621.6284332514149,
                        0,
                        0,
                        1)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(
                        N5.instance,
                        N1.instance,
                        0.006931789004551,
                        -0.124967324063348,
                        -0.003115210783801,
                        -0.002924665828185,
                        0.169363227039459)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(
                new Transform3d(
                    Units.inchesToMeters(11.0),
                    Units.inchesToMeters(-11.75),
                    Units.inchesToMeters(9.182678),
                    new Rotation3d(
                        0, Units.degreesToRadians(-20.0), Units.degreesToRadians(-115.0))))
            .build();

    public static final Camera[] V0_FUNKY_CAMS = {
      new Camera(new CameraIOLimelight(V0_FUNKY_CENTER)),
      new Camera(new CameraIOLimelight(V0_FUNKY_LEFT)),
      new Camera(new CameraIOLimelight(V0_FUNKY_RIGHT))
    };
    public static final Camera[] V1_STACKUP_CAMS = {
      new Camera(new CameraIOLimelight(V1_STACKUP_CENTER)),
      new Camera(new CameraIOLimelight(V1_STACKUP_LEFT)),
      new Camera(new CameraIOLimelight(V1_STACKUP_RIGHT))
    };
    public static final Camera[] V2_REDUNDANCY_CAMS = {
      new Camera(new CameraIOLimelight(V2_REDUNDANCY_CENTER)),
      new Camera(new CameraIOLimelight(V2_REDUNDANCY_LEFT)),
      new Camera(new CameraIOLimelight(V2_REDUNDANCY_RIGHT))
    };
    public static final Camera[] V3_EPSILON_CAMS = {
      new Camera(
          new CameraIOGompeiVision(
              BACK_BOTTOM_LEFT,
              () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark))),
        new Camera(
            new CameraIOGompeiVision(
                BACK_RIGHT,
                () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark))),
        new Camera(
            new CameraIOGompeiVision(
                FRONT_RIGHT,
                () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark))),
      new Camera(
          new CameraIOGompeiVision(
              FRONT_LEFT,
              () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)))
    };
  }

  @Builder
  public record LimelightConfig(
      String key,
      CameraType cameraType,
      double horizontalFOV,
      double verticalFOV,
      double megatagXYStdev,
      double metatagThetaStdev,
      double megatag2XYStdev,
      List<CameraDuty> cameraDuties,
      Transform3d robotToCameraTransform) {}

  @Builder
  public record GompeiVisionConfig(
      String key,
      String hardwareID,
      CameraType cameraType,
      double exposure,
      double gain,
      int width,
      int height,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N5, N1> distortionCoefficients,
      double horizontalFOV,
      double verticalFOV,
      double singletagXYStdev,
      double thetaStdev,
      double multitagXYStdev,
      List<CameraDuty> cameraDuties,
      Transform3d robotToCameraTransform) {}
}
