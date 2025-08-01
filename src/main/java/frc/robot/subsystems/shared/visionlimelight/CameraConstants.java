package frc.robot.subsystems.shared.visionlimelight;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;

public class CameraConstants {
  public static final double BLINK_TIME = 0.067;

  public static class Limelight2PlusConstants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double COMPLEMENTARY_FILTER_SIGMA = 0.5;
  }

  public static class Limelight3Constants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    public static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  public static class Limelight3GConstants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    public static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  public static class Limelight4Constants {
    public static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    public static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    public static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    public static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  public static class RobotCameras {
    private static final Camera V0_FUNKY_CENTER =
        new Camera(
            new CameraIOLimelight("center", CameraType.LIMELIGHT_3G),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-center")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                0,
                .241,
                .2,
                new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(-90))));

    private static final Camera V0_FUNKY_LEFT =
        new Camera(
            new CameraIOLimelight("left", CameraType.LIMELIGHT_2_PLUS),
            Limelight2PlusConstants.HORIZONTAL_FOV,
            Limelight2PlusConstants.VERTICAL_FOV,
            Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-left")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                .284,
                0.1884,
                .22,
                new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(-135))));

    private static final Camera V0_FUNKY_RIGHT =
        new Camera(
            new CameraIOLimelight("right", CameraType.LIMELIGHT_2_PLUS),
            Limelight2PlusConstants.HORIZONTAL_FOV,
            Limelight2PlusConstants.VERTICAL_FOV,
            Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-right")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -.284,
                0.1883,
                .22,
                new Rotation3d(Units.degreesToRadians(180), 0, Units.degreesToRadians(-45))));

    private static final Camera V1_STACKUP_CENTER =
        new Camera(
            new CameraIOLimelight("center", CameraType.LIMELIGHT_3G),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-center")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -0.211842, 0.0, 0.226176, new Rotation3d(0, 0, Units.degreesToRadians(-180))));

    private static final Camera V1_STACKUP_LEFT =
        new Camera(
            new CameraIOLimelight("left", CameraType.LIMELIGHT_3G),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-left")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -0.203974,
                -0.281026,
                0.237475,
                new Rotation3d(0.0, 0, Units.degreesToRadians(225))));

    private static final Camera V1_STACKUP_RIGHT =
        new Camera(
            new CameraIOLimelight("right", CameraType.LIMELIGHT_3G),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-right")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -0.203974, 0.281026, 0.237475, new Rotation3d(0, 0, Units.degreesToRadians(-225))));

    private static final Camera V2_REDUNDANCY_CENTER =
        new Camera(
            new CameraIOLimelight("center", CameraType.LIMELIGHT_3G),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-center")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -0.211842,
                -0.004974,
                0.221448,
                new Rotation3d(0, 0, Units.degreesToRadians(-180))));

    private static final Camera V2_REDUNDANCY_LEFT =
        new Camera(
            new CameraIOLimelight("left", CameraType.LIMELIGHT_4),
            Limelight4Constants.HORIZONTAL_FOV,
            Limelight4Constants.VERTICAL_FOV,
            Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-left")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -0.204072,
                -0.280928,
                0.231125,
                new Rotation3d(0.0, 0, Units.degreesToRadians(225))));

    private static final Camera V2_REDUNDANCY_RIGHT =
        new Camera(
            new CameraIOLimelight("right", CameraType.LIMELIGHT_4),
            Limelight4Constants.HORIZONTAL_FOV,
            Limelight4Constants.VERTICAL_FOV,
            Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-right")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION),
            new Transform3d(
                -0.204072, 0.280928, 0.231125, new Rotation3d(0, 0, Units.degreesToRadians(-225))));

    public static final Camera[] V0_FUNKY_CAMS = {V0_FUNKY_CENTER, V0_FUNKY_LEFT, V0_FUNKY_RIGHT};
    public static final Camera[] V1_STACKUP_CAMS = {
      V1_STACKUP_CENTER, V1_STACKUP_LEFT, V1_STACKUP_RIGHT
    };
    public static final Camera[] V2_REDUNDANCY_CAMS = {
      V2_REDUNDANCY_CENTER, V2_REDUNDANCY_LEFT, V2_REDUNDANCY_RIGHT
    };
  }

  public static class ReplayCameras {}
}
