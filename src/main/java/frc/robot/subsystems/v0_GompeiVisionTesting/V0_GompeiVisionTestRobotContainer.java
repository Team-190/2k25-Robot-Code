package frc.robot.subsystems.v0_GompeiVisionTesting;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.RobotStateGV;
import frc.robot.subsystems.shared.visiongompeivision.CameraIOGompeiVision;
import frc.robot.subsystems.shared.visiongompeivision.Vision;

public class V0_GompeiVisionTestRobotContainer implements RobotContainer {
  private Vision vision;

  public V0_GompeiVisionTestRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V0_GOMPEIVISION_TEST:
        case V0_GOMPEIVISION_TEST_SIM:
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                  new CameraIOGompeiVision(0, "front_cam"));
          break;
        default:
          break;
      }
    }

    if (vision == null) {
      vision =
          new Vision(
              () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
              new CameraIOGompeiVision(0, "front_cam"));
    }
  }

  @Override
  public void robotPeriodic() {
    RobotStateGV.periodic();
  }

  @Override
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
