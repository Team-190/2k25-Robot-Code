package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryUtil {
  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   *
   * @param x The x coordinate of the translation
   * @param y The y coordinate of the translation
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  public static final boolean isZero(Pose2d pose) {
    return pose.getX() == 0.0 && pose.getY() == 0.0 && pose.getRotation().getDegrees() == 0.0;
  }

  public static final boolean isZero(Pose2d[] pose) {
    for (Pose2d p : pose) {
      if (!isZero(p)) {
        return false;
      }
    }
    return true;
  }

  public static final boolean isZero(Translation2d translation) {
    return translation.getX() == 0.0 && translation.getY() == 0.0;
  }

  public static final boolean isZero(Rotation2d rotation) {
    return rotation.getDegrees() == 0.0;
  }
}
