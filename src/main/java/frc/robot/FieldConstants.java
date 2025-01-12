package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import lombok.RequiredArgsConstructor;

public class FieldConstants {
  @RequiredArgsConstructor
  public enum AlignmentPoses {
    TAG_6(
        new Translation2d(0.15879872585220567, 0.38812383025452923),
        new Translation2d(-0.15879872585220567, 0.38812383025452923),
        Rotation2d.fromDegrees(0)),
    TAG_7(
        new Translation2d(0.15879872585220567, 0.38812383025452923),
        new Translation2d(-0.15879872585220567, 0.38812383025452923),
        Rotation2d.fromDegrees(0)),
    TAG_8(
        new Translation2d(0.15879872585220567, 0.38812383025452923),
        new Translation2d(-0.15879872585220567, 0.38812383025452923),
        Rotation2d.fromDegrees(0)),
    TAG_9(
        new Translation2d(0.15879872585220567, 0.38812383025452923),
        new Translation2d(-0.15879872585220567, 0.38812383025452923),
        Rotation2d.fromDegrees(0)),
    TAG_10(
        new Translation2d(0.15879872585220567, 0.38812383025452923),
        new Translation2d(-0.15879872585220567, 0.38812383025452923),
        Rotation2d.fromDegrees(0)),
    TAG_11(
        new Translation2d(0.15879872585220567, 0.38812383025452923),
        new Translation2d(-0.15879872585220567, 0.38812383025452923),
        Rotation2d.fromDegrees(0));

    private final Translation2d leftTranslation;
    private final Translation2d rightTranslation;
    private final Rotation2d rotation;

    public Pose2d getPose(boolean left) {
      return new Pose2d(left ? leftTranslation : rightTranslation, rotation);
    }
  }

  public static final HashMap<Integer, AlignmentPoses> alignmentPoseMap =
      new HashMap<Integer, AlignmentPoses>();

  static {
    alignmentPoseMap.put(6, AlignmentPoses.TAG_6);
    alignmentPoseMap.put(7, AlignmentPoses.TAG_7);
    alignmentPoseMap.put(8, AlignmentPoses.TAG_8);
    alignmentPoseMap.put(9, AlignmentPoses.TAG_9);
    alignmentPoseMap.put(10, AlignmentPoses.TAG_10);
    alignmentPoseMap.put(11, AlignmentPoses.TAG_11);
    alignmentPoseMap.put(17, AlignmentPoses.TAG_6);
    alignmentPoseMap.put(18, AlignmentPoses.TAG_7);
    alignmentPoseMap.put(19, AlignmentPoses.TAG_8);
    alignmentPoseMap.put(20, AlignmentPoses.TAG_9);
    alignmentPoseMap.put(21, AlignmentPoses.TAG_10);
    alignmentPoseMap.put(22, AlignmentPoses.TAG_11);
  }
}
