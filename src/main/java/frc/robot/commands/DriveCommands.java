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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.InternalLoggedTracer;
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

    alignXController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().tolerance().get());
    alignYController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().tolerance().get());

    alignXController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.xPIDConstants().tolerance().get());
    alignYController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.yPIDConstants().tolerance().get());

    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    alignHeadingController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());
    alignHeadingController.setTolerance(
        DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.omegaPIDConstants().tolerance().get());

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
      BooleanSupplier bargeAlign,
      BooleanSupplier climbSpeed) {
    return Commands.run(
        () -> {
          ExternalLoggedTracer.reset();
          InternalLoggedTracer.reset();
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  DriveConstants.DRIVER_DEADBAND);
          InternalLoggedTracer.record(
              "Linear Magnitude", "Command Scheduler/Drive Commands/Joystick Drive");

          InternalLoggedTracer.reset();
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          InternalLoggedTracer.record(
              "Linear Direction", "Command Scheduler/Drive Commands/Joystick Drive");

          InternalLoggedTracer.reset();
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.DRIVER_DEADBAND);
          InternalLoggedTracer.record("Omega", "Command Scheduler/Drive Commands/Joystick Drive");

          // Square values
          InternalLoggedTracer.reset();
          linearMagnitude = linearMagnitude * linearMagnitude;
          InternalLoggedTracer.record("Square", "Command Scheduler/Drive Commands/Joystick Drive");

          // Calcaulate new linear velocity
          InternalLoggedTracer.reset();
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();
          InternalLoggedTracer.record(
              "Linear Velocity", "Command Scheduler/Drive Commands/Joystick Drive");

          // Get robot relative vel
          InternalLoggedTracer.reset();
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          InternalLoggedTracer.record(
              "Flipped?", "Command Scheduler/Drive Commands/Joystick Drive");

          InternalLoggedTracer.reset();
          double fieldRelativeXVel =
              linearVelocity.getX()
                  * DriveConstants.DRIVE_CONFIG.maxLinearVelocityMetersPerSecond();
          double fieldRelativeYVel =
              linearVelocity.getY()
                  * DriveConstants.DRIVE_CONFIG.maxLinearVelocityMetersPerSecond();
          InternalLoggedTracer.record(
              "Field Relative Velocity", "Command Scheduler/Drive Commands/Joystick Drive");

          InternalLoggedTracer.reset();
          double angular = 0.0;

          angular =
              (bargeAlign.getAsBoolean())
                  ? bargeAlignTheta()
                  : climbSpeed.getAsBoolean()
                      ? climberAlignTheta()
                      : rotateToReef.getAsBoolean()
                          ? reefThetaSpeedCalculate()
                          : omega * DriveConstants.DRIVE_CONFIG.maxAngularVelocity();
          InternalLoggedTracer.record("Angular", "Command Scheduler/Drive Commands/Joystick Drive");

          InternalLoggedTracer.reset();
          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  climbSpeed.getAsBoolean() ? fieldRelativeXVel * 0.25 : fieldRelativeXVel,
                  climbSpeed.getAsBoolean() ? fieldRelativeYVel * 0.25 : fieldRelativeYVel,
                  angular,
                  isFlipped
                      ? RobotState.getRobotPoseField().getRotation().plus(new Rotation2d(Math.PI))
                      : RobotState.getRobotPoseField().getRotation());
          InternalLoggedTracer.record(
              "Chassis Speeds", "Command Scheduler/Drive Commands/Joystick Drive");

          InternalLoggedTracer.reset();
          Logger.recordOutput("Drive/JoystickDrive/xSpeed", chassisSpeeds.vxMetersPerSecond);
          Logger.recordOutput("Drive/JoystickDrive/ySpeed", chassisSpeeds.vyMetersPerSecond);
          Logger.recordOutput(
              "Drive/JoystickDrive/thetaSpeed", chassisSpeeds.omegaRadiansPerSecond);
          InternalLoggedTracer.record("Logging", "Command Scheduler/Drive Commands/Joystick Drive");
          // Convert to field relative speeds & send command

          InternalLoggedTracer.reset();
          drive.runVelocity(chassisSpeeds);
          InternalLoggedTracer.record(
              "Apply Speeds", "Command Scheduler/Drive Commands/Joystick Drive");
          ExternalLoggedTracer.record(
              "Joystick Drive Total Time", "Command Scheduler/Drive Commands/Joystick Drive");
        },
        drive);
  }

  public static final Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier rotateToReef) {
    return joystickDrive(
        drive, xSupplier, ySupplier, omegaSupplier, rotateToReef, () -> false, () -> false);
  }

  private static double bargeAlignTheta() {
    ExternalLoggedTracer.reset();
    double thetaSpeed = 0.0;

    if (!alignHeadingController.atSetpoint())
      thetaSpeed =
          alignHeadingController.calculate(
              RobotState.getRobotPoseReef().getRotation().getRadians(),
              RobotState.getRobotPoseField().getX() < FieldConstants.fieldLength / 2 ? 0 : Math.PI);
    else alignHeadingController.reset(RobotState.getRobotPoseReef().getRotation().getRadians());

    Logger.recordOutput("Drive/thetaSpeed", thetaSpeed);
    ExternalLoggedTracer.record("Barge Align Theta", "Command Scheduler/Drive Commands");

    return thetaSpeed;
  }

  private static double climberAlignTheta() {
    ExternalLoggedTracer.reset();
    double thetaSpeed = 0.0;

    if (!alignHeadingController.atSetpoint())
      thetaSpeed =
          alignHeadingController.calculate(
              RobotState.getRobotPoseReef().getRotation().getRadians(),
              !AllianceFlipUtil.shouldFlip() ? 0 : Math.PI);
    else alignHeadingController.reset(RobotState.getRobotPoseReef().getRotation().getRadians());

    Logger.recordOutput("Drive/thetaSpeed", thetaSpeed);
    ExternalLoggedTracer.record("Barge Align Theta", "Command Scheduler/Drive Commands");

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
    return Commands.runOnce(
            () -> {
              RobotState.setAutoAligning(true);
            })
        .andThen(
            Commands.run(
                    () -> {
                      ExternalLoggedTracer.reset();
                      InternalLoggedTracer.reset();
                      ChassisSpeeds speeds;
                      InternalLoggedTracer.record(
                          "Create ChassisSpeeds",
                          "Command Scheduler/Drive Commands/Auto Align Coral");

                      if (RobotState.getReefAlignData().closestReefTag() != -1) {
                        InternalLoggedTracer.reset();
                        double xSpeed = 0.0;
                        double ySpeed = 0.0;
                        InternalLoggedTracer.record(
                            "Create Speeds", "Command Scheduler/Drive Commands/Auto Align Coral");

                        InternalLoggedTracer.reset();
                        double ex =
                            RobotState.getReefAlignData().coralSetpoint().getX()
                                - RobotState.getRobotPoseReef().getX();
                        double ey =
                            RobotState.getReefAlignData().coralSetpoint().getY()
                                - RobotState.getRobotPoseReef().getY();
                        InternalLoggedTracer.record(
                            "Create Cartesian Errors",
                            "Command Scheduler/Drive Commands/Auto Align Coral");

                        // Rotate errors into the reef post's coordinate frame
                        InternalLoggedTracer.reset();
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
                        InternalLoggedTracer.record(
                            "Create Rotated Errors",
                            "Command Scheduler/Drive Commands/Auto Align Coral");

                        if (!alignXController.atSetpoint()) {
                          InternalLoggedTracer.reset();
                          xSpeed = alignXController.calculate(0, ex_prime);
                          InternalLoggedTracer.record(
                              "Create XSpeed", "Command Scheduler/Drive Commands/Auto Align Algae");
                        } else {
                          InternalLoggedTracer.reset();
                          alignXController.reset(ex_prime);
                          InternalLoggedTracer.record(
                              "Reset XSpeed", "Command Scheduler/Drive Commands/Auto Align Algae");
                        }
                        if (!alignYController.atSetpoint()) {
                          InternalLoggedTracer.reset();
                          ySpeed = alignYController.calculate(0, ey_prime);
                          InternalLoggedTracer.record(
                              "Create YSpeed", "Command Scheduler/Drive Commands/Auto Align Algae");

                        } else {
                          InternalLoggedTracer.reset();
                          alignYController.reset(ey_prime);
                          InternalLoggedTracer.record(
                              "Reset YSpeed", "Command Scheduler/Drive Commands/Auto Align Algae");
                        }

                        // Re-rotate the speeds into field relative coordinate frame
                        InternalLoggedTracer.reset();
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
                        InternalLoggedTracer.record(
                            "Create Adjusted Speeds",
                            "Command Scheduler/Drive Commands/Auto Align Coral");

                        InternalLoggedTracer.reset();
                        speeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -adjustedXSpeed,
                                -adjustedYSpeed,
                                reefThetaSpeedCalculate(),
                                RobotState.getRobotPoseReef()
                                    .getRotation()
                                    .plus(new Rotation2d(Math.PI)));
                        InternalLoggedTracer.record(
                            "Update Populated ChassisSpeeds",
                            "Command Scheduler/Drive Commands/Auto Align Coral");

                      } else {
                        InternalLoggedTracer.reset();
                        speeds = new ChassisSpeeds();
                        InternalLoggedTracer.record(
                            "Updated Unpopulated ChassisSpeeds",
                            "Command Scheduler/Drive Commands");
                      }
                      InternalLoggedTracer.reset();
                      Logger.recordOutput("Drive/Coral/xSpeed", -speeds.vxMetersPerSecond);
                      Logger.recordOutput("Drive/Coral/ySpeed", -speeds.vyMetersPerSecond);
                      Logger.recordOutput("Drive/Coral/thetaSpeed", speeds.omegaRadiansPerSecond);
                      InternalLoggedTracer.record(
                          "Logging", "Command Scheduler/Drive Commands/Auto Align Coral");

                      InternalLoggedTracer.reset();
                      drive.runVelocity(speeds);
                      InternalLoggedTracer.record(
                          "Apply Speeds", "Command Scheduler/Drive Commands/Auto Align Coral");
                    },
                    drive)
                .until(() -> RobotState.getReefAlignData().atCoralSetpoint())
                .finallyDo(
                    () -> {
                      InternalLoggedTracer.reset();
                      drive.runVelocity(new ChassisSpeeds());
                      alignHeadingController.reset(
                          RobotState.getRobotPoseReef().getRotation().getRadians());
                      alignXController.reset(RobotState.getRobotPoseReef().getX());
                      alignYController.reset(RobotState.getRobotPoseReef().getY());
                      RobotState.setAutoAligning(false);
                      InternalLoggedTracer.record(
                          "Auto Align Coral End",
                          "Command Scheduler/Drive Commands/Auto Align Coral");
                      ExternalLoggedTracer.record(
                          "Total Time Auto Align Coral",
                          "Command Scheduler/Drive Commands/Auto Align Coral");
                    }));
  }

  public static Command autoAlignReefAlgae(Drive drive, Camera... cameras) {
    return Commands.runOnce(
            () -> {
              RobotState.setAutoAligning(true);
            })
        .andThen(
            Commands.run(
                    () -> {
                      ExternalLoggedTracer.reset();
                      InternalLoggedTracer.reset();
                      ChassisSpeeds speeds;
                      InternalLoggedTracer.record(
                          "Generate ChassisSpeeds",
                          "Command Scheduler/Drive Commands/Auto Align Algae");

                      if (RobotState.getReefAlignData().closestReefTag() != -1) {
                        InternalLoggedTracer.reset();
                        double xSpeed = 0.0;
                        double ySpeed = 0.0;
                        InternalLoggedTracer.record(
                            "Create Speeds", "Command Scheduler/Drive Commands/Auto Align Algae");

                        InternalLoggedTracer.reset();
                        double ex =
                            RobotState.getReefAlignData().algaeSetpoint().getX()
                                - RobotState.getRobotPoseReef().getX();
                        double ey =
                            RobotState.getReefAlignData().algaeSetpoint().getY()
                                - RobotState.getRobotPoseReef().getY();
                        InternalLoggedTracer.record(
                            "Create Cartesian Errors",
                            "Command Scheduler/Drive Commands/Auto Align Algae");

                        // Rotate errors into the reef post's coordinate frame
                        InternalLoggedTracer.reset();
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
                        InternalLoggedTracer.record(
                            "Create Rotated Errors",
                            "Command Scheduler/Drive Commands/Auto Align Algae");

                        if (!alignXController.atSetpoint()) {
                          InternalLoggedTracer.reset();
                          xSpeed = alignXController.calculate(0, ex_prime);
                          InternalLoggedTracer.record(
                              "Create XSpeed", "Command Scheduler/Drive Commands/Auto Align Algae");
                        } else {
                          InternalLoggedTracer.reset();
                          alignXController.reset(ex_prime);
                          InternalLoggedTracer.record(
                              "Reset XSpeed", "Command Scheduler/Drive Commands/Auto Align Algae");
                        }
                        if (!alignYController.atSetpoint()) {
                          InternalLoggedTracer.reset();
                          ySpeed = alignYController.calculate(0, ey_prime);
                          InternalLoggedTracer.record(
                              "Create YSpeed", "Command Scheduler/Drive Commands/Auto Align Algae");

                        } else {
                          InternalLoggedTracer.reset();
                          alignYController.reset(ey_prime);
                          InternalLoggedTracer.record(
                              "Reset YSpeed", "Command Scheduler/Drive Commands/Auto Align Algae");
                        }

                        // Re-rotate the speeds into field relative coordinate frame
                        InternalLoggedTracer.reset();
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

                        InternalLoggedTracer.record(
                            "Create Adjusted Speeds",
                            "Command Scheduler/Drive Commands/Auto Align Algae");

                        InternalLoggedTracer.reset();
                        speeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -adjustedXSpeed,
                                -adjustedYSpeed,
                                reefThetaSpeedCalculate(),
                                RobotState.getRobotPoseReef()
                                    .getRotation()
                                    .plus(new Rotation2d(Math.PI)));
                        InternalLoggedTracer.record(
                            "Update Populated Speeds",
                            "Command Scheduler/Drive Commands/Auto Align Algae");
                      } else {
                        InternalLoggedTracer.reset();
                        speeds = new ChassisSpeeds();
                        InternalLoggedTracer.record(
                            "Update Unpopulated Speeds",
                            "Command Scheduler/Drive Commands/Auto Align Algae");
                      }

                      InternalLoggedTracer.reset();
                      Logger.recordOutput("Drive/Algae/xSpeed", -speeds.vxMetersPerSecond);
                      Logger.recordOutput("Drive/Algae/ySpeed", -speeds.vyMetersPerSecond);
                      Logger.recordOutput("Drive/Algae/thetaSpeed", speeds.omegaRadiansPerSecond);
                      InternalLoggedTracer.record(
                          "Logging", "Command Scheduler/Drive Commands/Auto Align Algae");

                      InternalLoggedTracer.reset();
                      drive.runVelocity(speeds);
                      InternalLoggedTracer.record(
                          "Apply Speeds", "Command Scheduler/Drive Commands/Auto Align Algae");
                    },
                    drive)
                .until(() -> RobotState.getReefAlignData().atAlgaeSetpoint())
                .finallyDo(
                    () -> {
                      InternalLoggedTracer.reset();
                      drive.runVelocity(new ChassisSpeeds());
                      alignHeadingController.reset(
                          RobotState.getRobotPoseReef().getRotation().getRadians());
                      alignXController.reset(RobotState.getRobotPoseReef().getX());
                      alignYController.reset(RobotState.getRobotPoseReef().getY());
                      RobotState.setAutoAligning(false);
                      InternalLoggedTracer.record(
                          "Auto Align Algae End",
                          "Command Scheduler/Drive Commands/Auto Align Algae");
                      ExternalLoggedTracer.record(
                          "Auto Align Algae Total Time",
                          "Command Scheduler/Drive Commands/Auto Align Algae");
                    }));
  }

  private static double reefThetaSpeedCalculate() {
    InternalLoggedTracer.reset();
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
    InternalLoggedTracer.record("Reef Align Theta", "Command Scheduler/Drive Commands");

    return thetaSpeed;
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
