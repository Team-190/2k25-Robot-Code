package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import java.util.HashMap;
import java.util.Map;

public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static enum ReefPose {
      RIGHT,
      LEFT,
      ALGAE,
      CENTER
    }

    public static enum ReefHeight {
      STOW,
      CORAL_INTAKE,
      ALGAE_FLOOR_INTAKE,
      ALGAE_MID,
      ASS_TOP,
      ASS_BOT,
      ALGAE_INTAKE_TOP,
      ALGAE_INTAKE_BOTTOM,
      L1,
      L2,
      L3,
      L4,
      L4_PLUS,
      ALGAE_SCORE
    }

    public static record FaceSetpoints(Pose2d right, Pose2d left, Pose2d algae, Pose2d center) {
      public Pose2d getPostSetpoint(ReefPose post) {
        return post == ReefPose.LEFT ? left : post == ReefPose.RIGHT ? right : center;
      }

      public Pose2d getAlgaeSetpoint() {
        return algae;
      }
    }

    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order

    public static final Map<Integer, FaceSetpoints> reefMap = new HashMap<Integer, FaceSetpoints>();

    static {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      double adjustYBranch =
          Units.inchesToMeters(6.469); // Offset Y setpoint by center of tag to reef branch
      double adjustXBranch =
          DriveConstants.DRIVE_CONFIG.bumperWidth()
              / 2.0; // Offset X setpoint by center of robot to bumper

      //   double adjustYAlgae =
      //       Units.inchesToMeters(3.5); // Offset Y setpoint by center of tag to algae setpoint
      double adjustXAlgae =
          DriveConstants.DRIVE_CONFIG.bumperWidth() / 2.0
              + Units.inchesToMeters(2); // Offset X setpoint by center of robot to bumper

      reefMap.put(
          18,
          new FaceSetpoints(
              centerFaces[0].transformBy(
                  new Transform2d(adjustXBranch, adjustYBranch, new Rotation2d())),
              centerFaces[0].transformBy(
                  new Transform2d(adjustXBranch, -adjustYBranch, new Rotation2d())),
              centerFaces[0].transformBy(new Transform2d(adjustXAlgae, 0.0, new Rotation2d())),
              centerFaces[0].transformBy(new Transform2d(adjustXBranch, 0, new Rotation2d()))));
      reefMap.put(
          19,
          new FaceSetpoints(
              centerFaces[1].transformBy(
                  new Transform2d(adjustXBranch, adjustYBranch, new Rotation2d())),
              centerFaces[1].transformBy(
                  new Transform2d(adjustXBranch, -adjustYBranch, new Rotation2d())),
              centerFaces[1].transformBy(new Transform2d(adjustXAlgae, 0.0, new Rotation2d())),
              centerFaces[1].transformBy(new Transform2d(adjustXBranch, 0, new Rotation2d()))));
      reefMap.put(
          20,
          new FaceSetpoints(
              centerFaces[2].transformBy(
                  new Transform2d(adjustXBranch, adjustYBranch, new Rotation2d())),
              centerFaces[2].transformBy(
                  new Transform2d(adjustXBranch, -adjustYBranch, new Rotation2d())),
              centerFaces[2].transformBy(new Transform2d(adjustXAlgae, 0.0, new Rotation2d())),
              centerFaces[2].transformBy(new Transform2d(adjustXBranch, 0, new Rotation2d()))));
      reefMap.put(
          21,
          new FaceSetpoints(
              centerFaces[3].transformBy(
                  new Transform2d(adjustXBranch, adjustYBranch, new Rotation2d())),
              centerFaces[3].transformBy(
                  new Transform2d(adjustXBranch, -adjustYBranch, new Rotation2d())),
              centerFaces[3].transformBy(new Transform2d(adjustXAlgae, 0.0, new Rotation2d())),
              centerFaces[3].transformBy(new Transform2d(adjustXBranch, 0, new Rotation2d()))));
      reefMap.put(
          22,
          new FaceSetpoints(
              centerFaces[4].transformBy(
                  new Transform2d(adjustXBranch, adjustYBranch, new Rotation2d())),
              centerFaces[4].transformBy(
                  new Transform2d(adjustXBranch, -adjustYBranch, new Rotation2d())),
              centerFaces[4].transformBy(new Transform2d(adjustXAlgae, 0.0, new Rotation2d())),
              centerFaces[4].transformBy(new Transform2d(adjustXBranch, 0, new Rotation2d()))));
      reefMap.put(
          17,
          new FaceSetpoints(
              centerFaces[5].transformBy(
                  new Transform2d(adjustXBranch, adjustYBranch, new Rotation2d())),
              centerFaces[5].transformBy(
                  new Transform2d(adjustXBranch, -adjustYBranch, new Rotation2d())),
              centerFaces[5].transformBy(new Transform2d(adjustXAlgae, 0.0, new Rotation2d())),
              centerFaces[5].transformBy(new Transform2d(adjustXBranch, 0, new Rotation2d()))));
      reefMap.put(
          7,
          new FaceSetpoints(
              AllianceFlipUtil.overrideApply(reefMap.get(18).right),
              AllianceFlipUtil.overrideApply(reefMap.get(18).left),
              AllianceFlipUtil.overrideApply(reefMap.get(18).algae),
              AllianceFlipUtil.overrideApply(reefMap.get(18).center)));
      reefMap.put(
          6,
          new FaceSetpoints(
              AllianceFlipUtil.overrideApply(reefMap.get(19).right),
              AllianceFlipUtil.overrideApply(reefMap.get(19).left),
              AllianceFlipUtil.overrideApply(reefMap.get(19).algae),
              AllianceFlipUtil.overrideApply(reefMap.get(19).center)));
      reefMap.put(
          11,
          new FaceSetpoints(
              AllianceFlipUtil.overrideApply(reefMap.get(20).right),
              AllianceFlipUtil.overrideApply(reefMap.get(20).left),
              AllianceFlipUtil.overrideApply(reefMap.get(20).algae),
              AllianceFlipUtil.overrideApply(reefMap.get(20).center)));
      reefMap.put(
          10,
          new FaceSetpoints(
              AllianceFlipUtil.overrideApply(reefMap.get(21).right),
              AllianceFlipUtil.overrideApply(reefMap.get(21).left),
              AllianceFlipUtil.overrideApply(reefMap.get(21).algae),
              AllianceFlipUtil.overrideApply(reefMap.get(21).center)));
      reefMap.put(
          9,
          new FaceSetpoints(
              AllianceFlipUtil.overrideApply(reefMap.get(22).right),
              AllianceFlipUtil.overrideApply(reefMap.get(22).left),
              AllianceFlipUtil.overrideApply(reefMap.get(22).algae),
              AllianceFlipUtil.overrideApply(reefMap.get(22).center)));
      reefMap.put(
          8,
          new FaceSetpoints(
              AllianceFlipUtil.overrideApply(reefMap.get(17).right),
              AllianceFlipUtil.overrideApply(reefMap.get(17).left),
              AllianceFlipUtil.overrideApply(reefMap.get(17).algae),
              AllianceFlipUtil.overrideApply(reefMap.get(17).center)));
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public static final int[] validTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
}
