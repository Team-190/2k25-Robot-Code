// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
public class AllianceFlipUtil {
  public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
  public static double fieldLength = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          fieldLength - pose.getX(),
          fieldWidth - pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  /** Returns whether the driver station is reporting that the robot is on the red alliance */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
