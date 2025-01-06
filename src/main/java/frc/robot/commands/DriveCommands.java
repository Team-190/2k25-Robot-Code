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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public final class DriveCommands {
  @Getter private static final PIDController xController;
  @Getter private static final PIDController yController;
  @Getter private static final PIDController headingController;

  static {
    headingController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp().get(),
            0,
            DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd().get(),
            Constants.LOOP_PERIOD_SECONDS);

    xController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kp().get(),
            0.0,
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kd().get());
    yController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kp().get(),
            0.0,
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kd().get());

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(Units.degreesToRadians(1.0));
  }
  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static final Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  DriveConstants.DRIVER_DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.DRIVER_DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Get robot relative vel
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          double fieldRelativeXVel =
              linearVelocity.getX()
                  * DriveConstants.DRIVE_CONFIG.maxLinearVelocityMetersPerSecond();
          double fieldRelativeYVel =
              linearVelocity.getY()
                  * DriveConstants.DRIVE_CONFIG.maxLinearVelocityMetersPerSecond();

          double angular = 0.0;

          angular = omega * DriveConstants.DRIVE_CONFIG.maxAngularVelocity();

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeXVel,
                  fieldRelativeYVel,
                  angular,
                  isFlipped
                      ? RobotState.getRobotPose().getRotation().plus(new Rotation2d(Math.PI))
                      : RobotState.getRobotPose().getRotation());

          // Convert to field relative speeds & send command
          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static final Command stop(Drive drive) {
    return Commands.run(() -> drive.stopWithX());
  }

  public static final void setRotationPID(double kp, double kd) {
    headingController.setPID(kp, 0.0, kd);
  }

  public static final void setTranslationPID(double kp, double kd) {
    xController.setPID(kp, 0.0, kd);
    yController.setPID(kp, 0.0, kd);
  }
}
