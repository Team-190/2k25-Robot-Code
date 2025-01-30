package frc.robot.subsystems.v1_gamma;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class V1_GammaMechanism3d {
  private static final double EXTENSION_0_HEIGHT = 0.0; // Meters off the ground

  private static final double MIN_EXTENSION_METERS = EXTENSION_0_HEIGHT; // Meters off the ground
  private static final double MAX_EXTENSION_METERS = 0.5;

  public static final Pose3d[] getPoses(double elevatorExtensionMeters, Rotation2d funnelAngle) {
    double extensionMeters =
        MathUtil.clamp(elevatorExtensionMeters, MIN_EXTENSION_METERS, MAX_EXTENSION_METERS);
    double extensionFraction =
        MathUtil.inverseInterpolate(MIN_EXTENSION_METERS, MAX_EXTENSION_METERS, extensionMeters);


    
    return new Pose3d[] {};
  }
}