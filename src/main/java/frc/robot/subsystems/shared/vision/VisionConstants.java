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

    private static final GompeiVisionConfig BACK_TOP_LEFT =
        GompeiVisionConfig.builder()
            .key("camera_backtopleft")
            .hardwareID("camera_backtopleft")
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
                        1624.2147990980468,
                        0.0,
                        678.5977326688123,
                        0.0,
                        1623.8010651757406,
                        581.2937963816543,
                        0.0,
                        0.0,
                        1.0)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(
                        N5.instance,
                        N1.instance,
                        0.0962739678996323,
                        -0.17425866758812222,
                        -0.0010256591695401809,
                        0.00203018525003567,
                        0.1499388749754604)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)))
            .build();

    private static final GompeiVisionConfig BACK_BOTTOM_LEFT =
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
                        N3.instance, N3.instance, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(N5.instance, N1.instance, 0.0, 0.0, 0.0, 0.0, 0.0)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)))
            .build();

    private static final GompeiVisionConfig BACK_RIGHT =
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
                        N3.instance, N3.instance, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(N5.instance, N1.instance, 0.0, 0.0, 0.0, 0.0, 0.0)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)))
            .build();

    private static final GompeiVisionConfig FRONT_RIGHT =
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
                        1624.2147990980468,
                        0.0,
                        678.5977326688123,
                        0.0,
                        1623.8010651757406,
                        581.2937963816543,
                        0.0,
                        0.0,
                        1.0)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(
                        N5.instance,
                        N1.instance,
                        0.0962739678996323,
                        -0.17425866758812222,
                        -0.0010256591695401809,
                        0.00203018525003567,
                        0.1499388749754604)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)))
            .build();

    private static final GompeiVisionConfig FRONT_LEFT =
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
                        N3.instance, N3.instance, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)))
            .distortionCoefficients(
                new Matrix<N5, N1>(
                    MatBuilder.fill(N5.instance, N1.instance, 0.0, 0.0, 0.0, 0.0, 0.0)))
            .verticalFOV(ThriftyCamConstants.VERTICAL_FOV)
            .singletagXYStdev(ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .thetaStdev(ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT)
            .multitagXYStdev(ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT)
            .cameraDuties(List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION))
            .robotToCameraTransform(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)))
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
              BACK_TOP_LEFT,
              () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark))),
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
