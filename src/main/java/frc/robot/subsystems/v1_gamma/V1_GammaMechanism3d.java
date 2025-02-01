package frc.robot.subsystems.v1_gamma;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class V1_GammaMechanism3d {
  private static final double ELEVATOR_STAGE_1_MIN_HEIGHT = 0.095250; // Meters off the ground
  private static final double ELEVATOR_STAGE_1_MAX_HEIGHT = 0.8509;
  private static final double ELEVATOR_CARRIAGE_MANIPULATOR_MIN_HEIGHT = 0.120650;
  private static final double ELEVATOR_CARRIAGE_MANIPULATOR_MAX_HEIGHT = 0.822325;

  private static final double MIN_EXTENSION_METERS = ELEVATOR_CARRIAGE_MANIPULATOR_MIN_HEIGHT;
  private static final double MAX_EXTENSION_METERS =
      ELEVATOR_STAGE_1_MAX_HEIGHT + ELEVATOR_CARRIAGE_MANIPULATOR_MAX_HEIGHT;

  private static final Pose3d ELEVATOR_STAGE_1 =
      new Pose3d(0.088900, 0, 0.095250, new Rotation3d());
  private static final Pose3d ELEVATOR_CARRIAGE_MANIPULATOR =
      new Pose3d(0.088900, 0.0, 0.120650, new Rotation3d());
  private static final Pose3d FUNNEL_LEFT =
      new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  private static final Pose3d FUNNEL_RIGHT =
      new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

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

    double stage1Height = ELEVATOR_STAGE_1_MIN_HEIGHT;
    double carriageHeight = ELEVATOR_CARRIAGE_MANIPULATOR_MIN_HEIGHT;

    // If extension is within the first stage's range, only move Stage 1
    if (extensionMeters <= ELEVATOR_STAGE_1_MAX_HEIGHT) {
      carriageHeight = extensionMeters;
    } else {
      // Stage 1 is fully extended, start moving the carriage
      double remainingExtension = extensionMeters - ELEVATOR_CARRIAGE_MANIPULATOR_MAX_HEIGHT;
      stage1Height = ELEVATOR_STAGE_1_MIN_HEIGHT + remainingExtension;
      carriageHeight = ELEVATOR_CARRIAGE_MANIPULATOR_MAX_HEIGHT + remainingExtension;
    }

    // Create transformed poses
    Pose3d ELEVATOR_STAGE_1_POSE =
        ELEVATOR_STAGE_1.transformBy(
            new Transform3d(0, 0, stage1Height - ELEVATOR_STAGE_1_MIN_HEIGHT, new Rotation3d()));
    Pose3d ELEVATOR_CARRIAGE_POSE =
        ELEVATOR_CARRIAGE_MANIPULATOR.transformBy(
            new Transform3d(
                0, 0, carriageHeight - ELEVATOR_CARRIAGE_MANIPULATOR_MIN_HEIGHT, new Rotation3d()));

    return new Pose3d[] {
      ELEVATOR_STAGE_1_POSE,
      ELEVATOR_CARRIAGE_POSE,
      FUNNEL_LEFT.rotateBy(new Rotation3d(0.0, 0.0, funnelAngle.unaryMinus().getRadians())),
      FUNNEL_RIGHT.rotateBy(new Rotation3d(0.0, 0.0, funnelAngle.getRadians()))
    };
  }
}
