package frc.robot.subsystems.v3_Poot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;

public class V3_PootMechanism3d {
  private static final double ELEVATOR_STAGE_1_MIN_HEIGHT = 0.095250; // Meters off the ground
  private static final double ELEVATOR_STAGE_1_MAX_HEIGHT = 0.7809;
  private static final double ELEVATOR_CARRIAGE_MANIPULATOR_MIN_HEIGHT = 0.120650;
  private static final double ELEVATOR_CARRIAGE_MANIPULATOR_MAX_HEIGHT = 0.877522325;

  private static final double MIN_EXTENSION_METERS = ELEVATOR_CARRIAGE_MANIPULATOR_MIN_HEIGHT;
  private static final double MAX_EXTENSION_METERS =
      ELEVATOR_STAGE_1_MAX_HEIGHT + ELEVATOR_CARRIAGE_MANIPULATOR_MAX_HEIGHT;

  private static final Pose3d ELEVATOR_STAGE_1 =
      new Pose3d(-0.177800, 0, 0.095250, new Rotation3d());
  private static final Pose3d ELEVATOR_CARRIAGE_MANIPULATOR =
      new Pose3d(-0.15381615, 0, 0.120650, new Rotation3d());

  public static final Pose3d[] getPoses(
      double elevatorExtensionMeters, Rotation2d intakeAngle, Rotation2d armAngle) {
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
            new Transform3d(0, 0, carriageHeight, new Rotation3d()));

    Logger.recordOutput(
        "Zero Poses",
        new Pose3d[] {
          new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()
        });

    return new Pose3d[] {
      new Pose3d(0.1875, 0, 0.1363, new Rotation3d(0.0, intakeAngle.getRadians(), 0.0)),
      ELEVATOR_STAGE_1_POSE,
      ELEVATOR_CARRIAGE_POSE,
      new Pose3d(-0.15381615, 0, 0.270855, new Rotation3d())
          .transformBy(
              new Transform3d(
                  0, 0, carriageHeight, new Rotation3d(armAngle.getRadians(), 0.0, 0.0))),
    };
  }
}
