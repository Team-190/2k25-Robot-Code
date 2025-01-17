package frc.robot.subsystems.shared.vision;

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

  public static class RobotCameras {
    private static final Camera v0_FunkyCenter =
        new Camera(
            new CameraIOLimelight(
                "center",
                CameraType.LIMELIGHT_3G),
            Limelight3GConstants.HORIZONTAL_FOV,
            Limelight3GConstants.VERTICAL_FOV,
            Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-center")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION));

    private static final Camera v0_FunkyLeft =
        new Camera(
            new CameraIOLimelight(
                "left",
                CameraType.LIMELIGHT_2_PLUS),
            Limelight2PlusConstants.HORIZONTAL_FOV,
            Limelight2PlusConstants.VERTICAL_FOV,
            Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-left")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION));

    private static final Camera v0_FunkyRight =
        new Camera(
            new CameraIOLimelight(
                "right",
                CameraType.LIMELIGHT_2_PLUS),
            Limelight2PlusConstants.HORIZONTAL_FOV,
            Limelight2PlusConstants.VERTICAL_FOV,
            Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
            Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
            NetworkTableInstance.getDefault()
                .getTable("limelight-right")
                .getDoubleArrayTopic("robot_orientation_set")
                .publish(),
            List.of(CameraDuty.FIELD_LOCALIZATION, CameraDuty.REEF_LOCALIZATION));

    public static final Camera[] v0_FunkyCams = {v0_FunkyCenter, v0_FunkyLeft, v0_FunkyRight};
  }

  public static class ReplayCameras {}
}
