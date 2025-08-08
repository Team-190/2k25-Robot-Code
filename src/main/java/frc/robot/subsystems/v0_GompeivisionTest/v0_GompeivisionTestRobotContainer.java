package frc.robot.subsystems.v0_GompeivisionTest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.shared.vision.VisionConstants.RobotCameras;

public class v0_GompeivisionTestRobotContainer implements RobotContainer {
  // Subsystems
  private Vision vision;

  public v0_GompeivisionTestRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V0_GOMPEIVISION_TEST:
        case V0_GOMPEIVISION_TEST_SIM:
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
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
