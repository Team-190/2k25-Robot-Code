package frc.robot.subsystems.shared.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import lombok.Getter;

public class Vision extends SubsystemBase {
  @Getter private final Camera[] cameras;
  @Getter private final Supplier<AprilTagFieldLayout> fieldLayoutSupplier;

  public Vision(Supplier<AprilTagFieldLayout> fieldLayoutSupplier, Camera... cameras) {
    this.cameras = cameras;
    this.fieldLayoutSupplier = fieldLayoutSupplier;

    NetworkTableInstance.getDefault().getTable("field");

    for (AprilTag tag : fieldLayoutSupplier.get().getTags()) {
      NetworkTableInstance.getDefault()
          .getTable("field")
          .getEntry("tag_" + tag.ID)
          .setDoubleArray(
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
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.periodic();
    }
  }
}
