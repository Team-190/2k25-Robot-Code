package frc.robot.subsystems.v2_Redundancy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyMechanism3d {
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
      new Pose3d(
          0.009102,
          -0.104775,
          0.603223,
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(-57.687084)));
  private static final Pose3d FUNNEL_RIGHT =
      new Pose3d(
          0.009102,
          0.104775,
          0.603223,
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(57.687084)));
  private static final Pose3d ALGAE_ARM = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
  private static final Pose3d ALGAE_INTAKE = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());

  /**
   * Calculates and returns an array of Pose3d objects representing the positions of various
   * components of the robot's elevator mechanism based on the given elevator extension and funnel
   * angle.
   *
   * @param elevatorExtensionMeters The extension of the elevator in meters. This value is clamped
   *     between MIN_EXTENSION_METERS and MAX_EXTENSION_METERS.
   * @param funnelAngle The angle of the funnel as a Rotation2d object.
   * @return An array of Pose3d objects representing the transformed poses of the elevator stage 1,
   *     elevator carriage, left funnel, and right funnel.
   */
  public static final Pose3d[] getPoses(
      double elevatorExtensionMeters,
      Rotation2d funnelAngle,
      Rotation2d armAngle,
      double intakePosition) {
    double extensionMeters =
        MathUtil.clamp(elevatorExtensionMeters, MIN_EXTENSION_METERS, MAX_EXTENSION_METERS);

    double stage1Height = ELEVATOR_STAGE_1_MIN_HEIGHT;
    double carriageHeight = ELEVATOR_CARRIAGE_MANIPULATOR_MIN_HEIGHT;

    // If extension is within the first stage's range, only move carriage
    if (extensionMeters <= ELEVATOR_STAGE_1_MAX_HEIGHT) {
      carriageHeight = extensionMeters;
    } else {
      // Carriage is fully extended, start moving stage 1
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

    Logger.recordOutput(
        "Zero Poses",
        new Pose3d[] {
          new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()
        });
    return new Pose3d[] {
      ELEVATOR_STAGE_1_POSE,
      ELEVATOR_CARRIAGE_POSE,
      FUNNEL_LEFT
          .transformBy(
              new Transform3d(
                  new Translation3d(),
                  new Rotation3d(Units.degreesToRadians(23.0), Units.degreesToRadians(-15.0), 0.0)))
          .transformBy(
              new Transform3d(
                  new Translation3d(), new Rotation3d(0.0, 0.0, funnelAngle.getRadians()))),
      FUNNEL_RIGHT
          .transformBy(
              new Transform3d(
                  new Translation3d(),
                  new Rotation3d(
                      Units.degreesToRadians(-23.0), Units.degreesToRadians(-15.0), 0.0)))
          .transformBy(
              new Transform3d(
                  new Translation3d(),
                  new Rotation3d(0.0, 0.0, funnelAngle.unaryMinus().getRadians()))),
      ALGAE_ARM.rotateBy(new Rotation3d(0.0, armAngle.getRadians(), 0.0)),
      ALGAE_INTAKE.transformBy(new Transform3d(intakePosition, 0.0, 0.0, new Rotation3d()))
    };
  }
}
