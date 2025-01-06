// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.vision.CameraConstants;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.util.LTNUpdater;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case GAMMA:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          break;
        case DELTA:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          break;
        case GAMMA_SIM:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          break;
        case DELTA_SIM:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          vision =
              new Vision(
                  CameraConstants.RobotCameras.LEFT_CAMERA,
                  CameraConstants.RobotCameras.RIGHT_CAMERA);
          break;
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision =
          new Vision(
              CameraConstants.ReplayCameras.LEFT_CAMERA,
              CameraConstants.ReplayCameras.RIGHT_CAMERA);
    }

    switch (Constants.ROBOT) {
      case GAMMA:
      case GAMMA_SIM:
        snapbackConfigureButtonBindings();
        break;
      case DELTA:
      case DELTA_SIM:
        whiplashConfigureButtonBindings();
        break;
    }
  }

  private void whiplashConfigureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
  }

  private void snapbackConfigureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
  }

  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getFieldRelativeVelocity(),
        drive.getModulePositions(),
        vision.getCameras(),
        false,
        false,
        false);
    switch (Constants.ROBOT) {
      case DELTA:
      case DELTA_SIM:
      default:
        LTNUpdater.updateDrive(drive);
        break;
      case GAMMA:
      case GAMMA_SIM:
        LTNUpdater.updateDrive(drive);
        break;
    }
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
