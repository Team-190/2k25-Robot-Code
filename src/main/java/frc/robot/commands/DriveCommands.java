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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.subsystems.shared.vision.CameraDuty;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {
  @Getter private static final PIDController alignXController;
  @Getter private static final PIDController alignYController;
  @Getter private static final PIDController alignHeadingController;

  @Getter private static final PIDController autoXController;
  @Getter private static final PIDController autoYController;
  @Getter private static final PIDController autoHeadingController;

  static {
    alignHeadingController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp().get(),
            0,
            DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd().get(),
            Constants.LOOP_PERIOD_SECONDS);
    alignXController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kp().get(),
            0.0,
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kd().get());
    alignYController =
        new PIDController(
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kp().get(),
            0.0,
            DriveConstants.AUTO_ALIGN_GAINS.translation_Kd().get());

    autoHeadingController =
        new PIDController(
            DriveConstants.AUTO_GAINS.rotation_Kp().get(),
            0.0,
            DriveConstants.AUTO_GAINS.rotation_Kd().get(),
            Constants.LOOP_PERIOD_SECONDS);
    autoXController = new PIDController(DriveConstants.AUTO_GAINS.translation_Kp().get(), 0.0, 0.0);
    autoYController =
        new PIDController(
            DriveConstants.AUTO_GAINS.translation_Kp().get(),
            0.0,
            DriveConstants.AUTO_GAINS.translation_Kd().get());

    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    alignHeadingController.setTolerance(Units.degreesToRadians(1.0));

    autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    autoHeadingController.setTolerance(Units.degreesToRadians(1.0));
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
                      ? RobotState.getRobotPoseField().getRotation().plus(new Rotation2d(Math.PI))
                      : RobotState.getRobotPoseField().getRotation());

          // Convert to field relative speeds & send command
          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static final Command inchMovement(Drive drive, double x) {
    final Double[] distanceBefore = {0.0};
    return Commands.run(
            () -> {
              if (distanceBefore[0] >= drive.getModulePositions()[0].distanceMeters + x) {
                drive.stop();
                return;
              }
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      5.0,
                      0.0,
                      0.0,
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? RobotState.getRobotPoseField()
                              .getRotation()
                              .plus(new Rotation2d(Math.PI))
                          : RobotState.getRobotPoseField().getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              distanceBefore[0] = drive.getModulePositions()[0].distanceMeters;
            });
  }

  public static final Command stop(Drive drive) {
    return Commands.run(() -> drive.stopWithX());
  }

  public static final void setRotationPID(double kp, double kd) {
    alignHeadingController.setPID(kp, 0.0, kd);
  }

  public static final void setTranslationPID(double kp, double kd) {
    alignXController.setPID(kp, 0.0, kd);
    alignYController.setPID(kp, 0.0, kd);
  }

  public static Command feedforwardCharacterization(Drive drive) {
    return new KSCharacterization(
        drive, drive::runCharacterization, drive::getFFCharacterizationVelocity);
  }

  public static Command wheelRadiusCharacterization(Drive drive) {
    double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRawGyroRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRawGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.DRIVE_CONFIG.driveBaseRadius())
                              / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  public static Command alignRobotToAprilTag(Drive drive, Camera... cameras) {

    ProfiledPIDController xController =
        new ProfiledPIDController(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().kP().get(),
            0.0,
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().kD().get(),
            new TrapezoidProfile.Constraints(
                DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS
                    .xPIDConstants()
                    .maxVelocity()
                    .get(),
                Double.POSITIVE_INFINITY));
    ProfiledPIDController yController =
        new ProfiledPIDController(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().kP().get(),
            0.0,
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().kD().get(),
            new TrapezoidProfile.Constraints(
                DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS
                    .yPIDConstants()
                    .maxVelocity()
                    .get(),
                Double.POSITIVE_INFINITY));
    ProfiledPIDController omegaController =
        new ProfiledPIDController(
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().kP().get(),
            0.0,
            DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().kD().get(),
            new TrapezoidProfile.Constraints(
                DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS
                    .omegaPIDConstants()
                    .maxVelocity()
                    .get(),
                Double.POSITIVE_INFINITY));

    xController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().tolerance().get());
    yController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().tolerance().get());
    omegaController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());

    return Commands.runOnce(
            () -> {
              for (Camera camera : cameras) {
                if (camera.getCameraDuties().contains(CameraDuty.REEF_LOCALIZATION)) {
                  camera.setValidTags(RobotState.getReefAlignData().closestReefTag());
                }
              }
            })
        .andThen(
            Commands.run(
                    () -> {
                      boolean isFlipped =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red;
                      ChassisSpeeds speeds;
                      if (RobotState.getReefAlignData().closestReefTag() != -1) {
                        double xSpeed = 0.0;
                        double ySpeed = 0.0;
                        double thetaSpeed = 0.0;
                        if (!xController.atSetpoint())
                          xSpeed =
                              xController.calculate(
                                  RobotState.getRobotPoseReef().getX(),
                                  RobotState.getReefAlignData().setpoint().getX());
                        else xController.reset(RobotState.getRobotPoseReef().getX());
                        if (!yController.atSetpoint())
                          ySpeed =
                              yController.calculate(
                                  RobotState.getRobotPoseReef().getY(),
                                  RobotState.getReefAlignData().setpoint().getY());
                        else yController.reset(RobotState.getRobotPoseReef().getY());
                        if (!omegaController.atSetpoint())
                          thetaSpeed =
                              omegaController.calculate(
                                  RobotState.getRobotPoseReef().getRotation().getRadians(),
                                  RobotState.getReefAlignData()
                                      .setpoint()
                                      .getRotation()
                                      .plus(Rotation2d.fromDegrees(-90.0))
                                      .getRadians());
                        else
                          omegaController.reset(
                              RobotState.getRobotPoseReef().getRotation().getRadians());

                        Logger.recordOutput("xSpeed", -xSpeed);
                        Logger.recordOutput("ySpeed", -ySpeed);
                        Logger.recordOutput("thetaSpeed", thetaSpeed);
                        speeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -xSpeed,
                                -ySpeed,
                                thetaSpeed,
                                isFlipped
                                    ? RobotState.getRobotPoseReef()
                                        .getRotation()
                                        .plus(new Rotation2d(Math.PI))
                                    : RobotState.getRobotPoseReef().getRotation());
                      } else {
                        speeds = new ChassisSpeeds();
                      }
                      drive.runVelocity(speeds);
                    },
                    drive)
                .until(() -> RobotState.getReefAlignData().atSetpoint())
                .finallyDo(
                    () -> {
                      drive.runVelocity(new ChassisSpeeds());
                      omegaController.reset(
                          RobotState.getRobotPoseReef().getRotation().getRadians());
                      xController.reset(RobotState.getRobotPoseReef().getX());
                      yController.reset(RobotState.getRobotPoseReef().getY());
                      for (Camera camera : cameras) {
                        camera.setValidTags(FieldConstants.validTags);
                      }
                    }));
  }

  public static double absMax(double a, double b) {
    double ans = Math.max(Math.abs(a), Math.abs(b));
    if (ans == Math.abs(a)) return Math.copySign(ans, a);
    else return Math.copySign(ans, b);
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
