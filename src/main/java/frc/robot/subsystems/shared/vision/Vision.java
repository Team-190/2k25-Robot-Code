package frc.robot.subsystems.shared.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  @Getter private final Camera[] cameras;
  @Getter private final Supplier<AprilTagFieldLayout> fieldLayoutSupplier;
  private final NetworkTable fieldTable;

  @Getter List<Pose2d> allRobotPoses;
  @Getter List<Pose2d> allTagPoses;

  public Vision(Supplier<AprilTagFieldLayout> fieldLayoutSupplier, Camera... cameras) {
    this.cameras = cameras;
    this.fieldLayoutSupplier = fieldLayoutSupplier;

    this.fieldTable = NetworkTableInstance.getDefault().getTable("field");

    for (AprilTag tag : fieldLayoutSupplier.get().getTags()) {
      this.fieldTable
          .getDoubleArrayTopic("tag_" + tag.ID)
          .publish()
          .set(
              new double[] {
                tag.pose.getX(),
                tag.pose.getY(),
                tag.pose.getZ(),
                tag.pose.getRotation().getQuaternion().getW(),
                tag.pose.getRotation().getQuaternion().getX(),
                tag.pose.getRotation().getQuaternion().getY(),
                tag.pose.getRotation().getQuaternion().getZ()
              });
    }

    allRobotPoses = new ArrayList<>();
    allTagPoses = new ArrayList<>();
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.periodic();
      addRobotPoses(camera.getRobotPose());
    }

    Logger.recordOutput("Vision/Field Table Connected", fieldTable.getInstance().isConnected());
    Logger.recordOutput("Vision/All Robot Poses", allRobotPoses.toArray(Pose2d[]::new));

    allRobotPoses.clear();
    allTagPoses.clear();
  }

  private void addRobotPoses(Pose2d... poses) {
    for (Pose2d pose : poses) {
      if (pose != null) {
        allRobotPoses.add(pose);
      }
    }
  }
}
