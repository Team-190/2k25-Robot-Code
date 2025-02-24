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
    public static enum ReefPost {
      LEFT,
      RIGHT
    }

    public static enum ReefHeight {
      STOW,
      INTAKE,
      L1,
      L2,
      L3,
      L4,
      TOP_ALGAE,
      BOT_ALGAE
    }

    public static record PostPair(Pose2d right, Pose2d left) {
      public Pose2d getPost(ReefPost post) {
        return post == ReefPost.LEFT ? left : right;
      }
    }

    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order

    public static final Map<Integer, PostPair> reefMap = new HashMap<Integer, PostPair>();

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

      double adjustY =
          Units.inchesToMeters(6.469); // Offset Y setpoint by center of tag to reef post
      double adjustX =
          DriveConstants.DRIVE_CONFIG.bumperWidth()
              / 2.0; // Offset X setpoint by center of robot to bumper

      reefMap.put(
          18,
          new PostPair(
              centerFaces[0].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
              centerFaces[0].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))));
      reefMap.put(
          19,
          new PostPair(
              centerFaces[1].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
              centerFaces[1].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))));
      reefMap.put(
          20,
          new PostPair(
              centerFaces[2].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
              centerFaces[2].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))));
      reefMap.put(
          21,
          new PostPair(
              centerFaces[3].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
              centerFaces[3].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))));
      reefMap.put(
          22,
          new PostPair(
              centerFaces[4].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
              centerFaces[4].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))));
      reefMap.put(
          17,
          new PostPair(
              centerFaces[5].transformBy(new Transform2d(adjustX, adjustY, new Rotation2d())),
              centerFaces[5].transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))));
      reefMap.put(
          7,
          new PostPair(
              AllianceFlipUtil.overrideApply(reefMap.get(18).right),
              AllianceFlipUtil.overrideApply(reefMap.get(18).left)));
      reefMap.put(
          6,
          new PostPair(
              AllianceFlipUtil.overrideApply(reefMap.get(19).right),
              AllianceFlipUtil.overrideApply(reefMap.get(19).left)));
      reefMap.put(
          11,
          new PostPair(
              AllianceFlipUtil.overrideApply(reefMap.get(20).right),
              AllianceFlipUtil.overrideApply(reefMap.get(20).left)));
      reefMap.put(
          10,
          new PostPair(
              AllianceFlipUtil.overrideApply(reefMap.get(21).right),
              AllianceFlipUtil.overrideApply(reefMap.get(21).left)));
      reefMap.put(
          9,
          new PostPair(
              AllianceFlipUtil.overrideApply(reefMap.get(22).right),
              AllianceFlipUtil.overrideApply(reefMap.get(22).left)));
      reefMap.put(
          8,
          new PostPair(
              AllianceFlipUtil.overrideApply(reefMap.get(17).right),
              AllianceFlipUtil.overrideApply(reefMap.get(17).left)));
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

  public static final int[] validTags = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22
  };
}
