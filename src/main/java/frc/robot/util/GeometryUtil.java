package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil {
  public static final boolean isZero(Pose2d pose) {
    return pose.getX() == 0.0 && pose.getY() == 0.0 && pose.getRotation().getDegrees() == 0.0;
  }

  public static final boolean isZero(Translation2d translation) {
    return translation.getX() == 0.0 && translation.getY() == 0.0;
  }

  public static final boolean isZero(Rotation2d rotation) {
    return rotation.getDegrees() == 0.0;
  }
}
