package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
  public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
  public static double fieldLength = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

  public static double applyX(double x) {
    return shouldFlip() ? fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static double overrideApplyX(double x) {
    return fieldLength - x;
  }

  public static double overrideApplyY(double y) {
    return fieldWidth - y;
  }

  public static Translation2d overrideApply(Translation2d translation) {
    return new Translation2d(
        overrideApplyX(translation.getX()), overrideApplyY(translation.getY()));
  }

  public static Rotation2d overrideApply(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static Pose2d overrideApply(Pose2d pose) {
    return new Pose2d(overrideApply(pose.getTranslation()), overrideApply(pose.getRotation()));
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
