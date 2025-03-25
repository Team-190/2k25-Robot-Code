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
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public final class DriveCommands {
  @Getter private static final ProfiledPIDController alignXController;
  @Getter private static final ProfiledPIDController alignYController;
  private static final ProfiledPIDController alignHeadingController;

  @Getter private static final PIDController autoXController;
  @Getter private static final PIDController autoYController;
  @Getter private static final PIDController autoHeadingController;

  static {
    alignXController =
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
    alignYController =
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
    alignHeadingController =
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
      DoubleSupplier omegaSupplier,
      BooleanSupplier rotateToReef,
      BooleanSupplier bargeAlign) {
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

          angular =
              bargeAlign.getAsBoolean()
                  ? bargeAlignTheta()
                  : rotateToReef.getAsBoolean()
                      ? reefThetaSpeedCalculate()
                      : omega * DriveConstants.DRIVE_CONFIG.maxAngularVelocity();

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeXVel,
                  bargeAlign.getAsBoolean() ? 0.0 : fieldRelativeYVel,
                  angular,
                  isFlipped
                      ? RobotState.getRobotPoseField().getRotation().plus(new Rotation2d(Math.PI))
                      : RobotState.getRobotPoseField().getRotation());
          Logger.recordOutput("Drive/JoystickDrive/xSpeed", chassisSpeeds.vxMetersPerSecond);
          Logger.recordOutput("Drive/JoystickDrive/ySpeed", chassisSpeeds.vyMetersPerSecond);
          Logger.recordOutput(
              "Drive/JoystickDrive/thetaSpeed", chassisSpeeds.omegaRadiansPerSecond);
          // Convert to field relative speeds & send command
          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static final Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier rotateToReef) {
    return joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, rotateToReef, () -> false);
  }

  private static double bargeAlignTheta() {
    double thetaSpeed = 0.0;

    alignHeadingController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());

    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);

    if (!alignHeadingController.atSetpoint())
      thetaSpeed =
          alignHeadingController.calculate(
              RobotState.getRobotPoseReef().getRotation().getRadians(),
              RobotState.getRobotPoseField().getX() < FieldConstants.fieldLength / 2 ? 0 : Math.PI);
    else alignHeadingController.reset(RobotState.getRobotPoseReef().getRotation().getRadians());

    Logger.recordOutput("Drive/thetaSpeed", thetaSpeed);
    return thetaSpeed;
  }

  public static final Command inchMovement(Drive drive, double velocity, double time) {
    return Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, velocity, 0.0)))
        .withTimeout(time);
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

  public static Command autoAlignReefCoral(Drive drive, Camera... cameras) {
    alignXController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().tolerance().get());
    alignYController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().tolerance().get());

    return Commands.runOnce(
            () -> {
              RobotState.setAutoAligning(true);
            })
        .andThen(
            Commands.run(
                    () -> {
                      ChassisSpeeds speeds;
                      if (RobotState.getReefAlignData().closestReefTag() != -1) {
                        double xSpeed = 0.0;
                        double ySpeed = 0.0;

                        double ex =
                            RobotState.getReefAlignData().coralSetpoint().getX()
                                - RobotState.getRobotPoseReef().getX();
                        double ey =
                            RobotState.getReefAlignData().coralSetpoint().getY()
                                - RobotState.getRobotPoseReef().getY();

                        // Rotate errors into the reef post's coordinate frame
                        double ex_prime =
                            ex
                                    * Math.cos(
                                        RobotState.getReefAlignData()
                                            .coralSetpoint()
                                            .getRotation()
                                            .getRadians())
                                + ey
                                    * Math.sin(
                                        RobotState.getReefAlignData()
                                            .coralSetpoint()
                                            .getRotation()
                                            .getRadians());
                        double ey_prime =
                            -ex
                                    * Math.sin(
                                        RobotState.getReefAlignData()
                                            .coralSetpoint()
                                            .getRotation()
                                            .getRadians())
                                + ey
                                    * Math.cos(
                                        RobotState.getReefAlignData()
                                            .coralSetpoint()
                                            .getRotation()
                                            .getRadians());

                        if (!alignXController.atSetpoint())
                          xSpeed = alignXController.calculate(0, ex_prime);
                        else alignXController.reset(ex_prime);
                        if (!alignYController.atSetpoint())
                          ySpeed = alignYController.calculate(0, ey_prime);
                        else alignYController.reset(ey_prime);

                        // Re-rotate the speeds into field relative coordinate frame
                        double adjustedXSpeed =
                            xSpeed
                                    * Math.cos(
                                        RobotState.getReefAlignData()
                                            .coralSetpoint()
                                            .getRotation()
                                            .getRadians())
                                - ySpeed
                                    * Math.sin(
                                        RobotState.getReefAlignData()
                                            .coralSetpoint()
                                            .getRotation()
                                            .getRadians());
                        double adjustedYSpeed =
                            xSpeed
                                    * Math.sin(
                                        RobotState.getReefAlignData()
                                            .coralSetpoint()
                                            .getRotation()
                                            .getRadians())
                                + ySpeed
                                    * Math.cos(
                                        RobotState.getReefAlignData()
                                            .coralSetpoint()
                                            .getRotation()
                                            .getRadians());

                        speeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -adjustedXSpeed,
                                -adjustedYSpeed,
                                reefThetaSpeedCalculate(),
                                RobotState.getRobotPoseReef()
                                    .getRotation()
                                    .plus(new Rotation2d(Math.PI)));
                      } else {
                        speeds = new ChassisSpeeds();
                      }
                      Logger.recordOutput("Drive/Coral/xSpeed", -speeds.vxMetersPerSecond);
                      Logger.recordOutput("Drive/Coral/ySpeed", -speeds.vyMetersPerSecond);
                      Logger.recordOutput("Drive/Coral/thetaSpeed", speeds.omegaRadiansPerSecond);
                      drive.runVelocity(speeds);
                    },
                    drive)
                .until(() -> RobotState.getReefAlignData().atCoralSetpoint())
                .finallyDo(
                    () -> {
                      drive.runVelocity(new ChassisSpeeds());
                      alignHeadingController.reset(
                          RobotState.getRobotPoseReef().getRotation().getRadians());
                      alignXController.reset(RobotState.getRobotPoseReef().getX());
                      alignYController.reset(RobotState.getRobotPoseReef().getY());
                      RobotState.setAutoAligning(false);
                    }));
  }

  public static Command autoAlignReefAlgae(Drive drive, Camera... cameras) {
    alignXController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().tolerance().get());
    alignYController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().tolerance().get());

    return Commands.runOnce(
            () -> {
              RobotState.setAutoAligning(true);
            })
        .andThen(
            Commands.run(
                    () -> {
                      ChassisSpeeds speeds;
                      if (RobotState.getReefAlignData().closestReefTag() != -1) {
                        double xSpeed = 0.0;
                        double ySpeed = 0.0;

                        double ex =
                            RobotState.getReefAlignData().algaeSetpoint().getX()
                                - RobotState.getRobotPoseReef().getX();
                        double ey =
                            RobotState.getReefAlignData().algaeSetpoint().getY()
                                - RobotState.getRobotPoseReef().getY();

                        // Rotate errors into the reef post's coordinate frame
                        double ex_prime =
                            ex
                                    * Math.cos(
                                        RobotState.getReefAlignData()
                                            .algaeSetpoint()
                                            .getRotation()
                                            .getRadians())
                                + ey
                                    * Math.sin(
                                        RobotState.getReefAlignData()
                                            .algaeSetpoint()
                                            .getRotation()
                                            .getRadians());
                        double ey_prime =
                            -ex
                                    * Math.sin(
                                        RobotState.getReefAlignData()
                                            .algaeSetpoint()
                                            .getRotation()
                                            .getRadians())
                                + ey
                                    * Math.cos(
                                        RobotState.getReefAlignData()
                                            .algaeSetpoint()
                                            .getRotation()
                                            .getRadians());

                        if (!alignXController.atSetpoint())
                          xSpeed = alignXController.calculate(0, ex_prime);
                        else alignXController.reset(ex_prime);
                        if (!alignYController.atSetpoint())
                          ySpeed = alignYController.calculate(0, ey_prime);
                        else alignYController.reset(ey_prime);

                        // Re-rotate the speeds into field relative coordinate frame
                        double adjustedXSpeed =
                            xSpeed
                                    * Math.cos(
                                        RobotState.getReefAlignData()
                                            .algaeSetpoint()
                                            .getRotation()
                                            .getRadians())
                                - ySpeed
                                    * Math.sin(
                                        RobotState.getReefAlignData()
                                            .algaeSetpoint()
                                            .getRotation()
                                            .getRadians());
                        double adjustedYSpeed =
                            xSpeed
                                    * Math.sin(
                                        RobotState.getReefAlignData()
                                            .algaeSetpoint()
                                            .getRotation()
                                            .getRadians())
                                + ySpeed
                                    * Math.cos(
                                        RobotState.getReefAlignData()
                                            .algaeSetpoint()
                                            .getRotation()
                                            .getRadians());

                        speeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -adjustedXSpeed,
                                -adjustedYSpeed,
                                reefThetaSpeedCalculate(),
                                RobotState.getRobotPoseReef()
                                    .getRotation()
                                    .plus(new Rotation2d(Math.PI)));
                      } else {
                        speeds = new ChassisSpeeds();
                      }
                      Logger.recordOutput("Drive/Algae/xSpeed", -speeds.vxMetersPerSecond);
                      Logger.recordOutput("Drive/Algae/ySpeed", -speeds.vyMetersPerSecond);
                      Logger.recordOutput("Drive/Algae/thetaSpeed", speeds.omegaRadiansPerSecond);
                      drive.runVelocity(speeds);
                    },
                    drive)
                .until(() -> RobotState.getReefAlignData().atAlgaeSetpoint())
                .finallyDo(
                    () -> {
                      drive.runVelocity(new ChassisSpeeds());
                      alignHeadingController.reset(
                          RobotState.getRobotPoseReef().getRotation().getRadians());
                      alignXController.reset(RobotState.getRobotPoseReef().getX());
                      alignYController.reset(RobotState.getRobotPoseReef().getY());
                      RobotState.setAutoAligning(false);
                    }));
  }

  private static double reefThetaSpeedCalculate() {
    double thetaSpeed = 0.0;

    alignHeadingController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());

    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);

    if (!alignHeadingController.atSetpoint())
      thetaSpeed =
          alignHeadingController.calculate(
              RobotState.getRobotPoseReef().getRotation().getRadians(),
              RobotState.getReefAlignData().coralSetpoint().getRotation().getRadians());
    else alignHeadingController.reset(RobotState.getRobotPoseReef().getRotation().getRadians());

    Logger.recordOutput("Drive/thetaSpeed", thetaSpeed);
    return thetaSpeed;
  }

  public static Command autoAlignBargeAlgae(Drive drive) {
    alignXController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().tolerance().get());
    alignYController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().tolerance().get());

    return Commands.runOnce(
            () -> {
              RobotState.setAutoAligning(true);
            })
        .andThen(
            Commands.run(
                    () -> {
                      ChassisSpeeds speeds;
                      double xSpeed = 0.0;
                      if (!alignXController.atSetpoint()) {
                        xSpeed =
                            alignXController.calculate(
                                RobotState.getRobotPoseField().getX(),
                                RobotState.getBargeAlignData().bargeSetpoint());
                      } else alignXController.reset(RobotState.getRobotPoseField().getX());
                      speeds =
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              -xSpeed,
                              0,
                              bargeThetaSpeedCalculate(),
                              RobotState.getRobotPoseReef()
                                  .getRotation()
                                  .plus(new Rotation2d(Math.PI)));
                      Logger.recordOutput("Drive/Barge/xSpeed", -speeds.vxMetersPerSecond);
                      Logger.recordOutput("Drive/Barge/ySpeed", -speeds.vyMetersPerSecond);
                      Logger.recordOutput("Drive/Barge/thetaSpeed", speeds.omegaRadiansPerSecond);
                      drive.runVelocity(speeds);
                    },
                    drive)
                .until(() -> RobotState.getBargeAlignData().atBargeSetpoint())
                .finallyDo(
                    () -> {
                      drive.runVelocity(new ChassisSpeeds());
                      alignHeadingController.reset(
                          RobotState.getRobotPoseReef().getRotation().getRadians());
                      alignXController.reset(RobotState.getRobotPoseReef().getX());
                      alignYController.reset(RobotState.getRobotPoseReef().getY());
                      RobotState.setAutoAligning(false);
                    }));
  }

  private static double bargeThetaSpeedCalculate() {
    double thetaSpeed = 0.0;

    alignHeadingController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());

    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);

    if (!alignHeadingController.atSetpoint())
      thetaSpeed =
          alignHeadingController.calculate(
              RobotState.getRobotPoseField().getRotation().getRadians(),
              RobotState.getRobotPoseField().getX() <= FieldConstants.fieldLength / 2
                  ? 0
                  : Math.PI);
    else alignHeadingController.reset(RobotState.getRobotPoseField().getRotation().getRadians());

    Logger.recordOutput("Drive/thetaSpeed", thetaSpeed);
    return thetaSpeed;
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
