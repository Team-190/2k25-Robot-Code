package frc.robot.subsystems.v3_Epsilon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.shared.vision.VisionConstants.RobotCameras;

public class V3_EpsilonRobotContainer implements RobotContainer {
  // Subsystems
  private Vision vision;

  public V3_EpsilonRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V3_EPSILON:
        case V3_EPSILON_SIM:
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                  RobotCameras.V3_EPSILON_CAMS);
          break;
        default:
          break;
      }
    }

    if (vision == null) {
      vision =
          new Vision(
              () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
              RobotCameras.V3_EPSILON_CAMS);
    }

    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {}

  private void configureAutos() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
