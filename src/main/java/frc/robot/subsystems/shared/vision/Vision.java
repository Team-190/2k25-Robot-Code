package frc.robot.subsystems.shared.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Vision extends SubsystemBase {
  @Getter private final Camera[] cameras;

  public Vision(Camera... cameras) {
    this.cameras = cameras;
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.periodic();
    }
  }
}
