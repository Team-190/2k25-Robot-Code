package frc.robot.subsystems.v1_gamma;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class V1_GammaMechanism3d {
  private static final double EXTENSION_0_HEIGHT = 0.0; // Meters off the ground

  private static final double MIN_EXTENSION_METERS = EXTENSION_0_HEIGHT; // Meters off the ground
  private static final double MAX_EXTENSION_METERS = 0.5;

  private static final Pose3d ELEVATOR_STAGE_1 =
      new Pose3d(0.088900, 0, 0.095250, new Rotation3d());
  private static final Pose3d ELEVATOR_CARRIAGE_MANIPULATOR =
      new Pose3d(0.088900, 0.0, 0.120650, new Rotation3d());
  private static final Pose3d FUNNEL_LEFT =
      new Pose3d(-0.009102, 0.104775, 0.593223, new Rotation3d(0.0, 0.0, 0.0));
  private static final Pose3d FUNNEL_RIGHT =
      new Pose3d(-0.009102, -0.104775, 0.593223, new Rotation3d(0.0, 0.0, 0.0));

  /*
   * Order:
   *   Elevator stage 1
   *   Elevator carriage and manipulator
   *   Funnel left
   *   Funnel right
   */
  public static final Pose3d[] getPoses(double elevatorExtensionMeters, Rotation2d funnelAngle) {
    double extensionMeters =
        MathUtil.clamp(elevatorExtensionMeters, MIN_EXTENSION_METERS, MAX_EXTENSION_METERS);
    double extensionFraction =
        MathUtil.inverseInterpolate(MIN_EXTENSION_METERS, MAX_EXTENSION_METERS, extensionMeters);

    return new Pose3d[] {
      ELEVATOR_STAGE_1,
      ELEVATOR_CARRIAGE_MANIPULATOR,
      FUNNEL_LEFT.rotateBy(new Rotation3d(0.0, 0.0, funnelAngle.unaryMinus().getRadians())),
      FUNNEL_RIGHT.rotateBy(new Rotation3d(0.0, 0.0, funnelAngle.getRadians()))
    };
  }
}
