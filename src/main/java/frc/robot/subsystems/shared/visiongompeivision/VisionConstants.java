// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shared.visiongompeivision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Builder;

public class VisionConstants {
  private static final boolean forceEnableInstanceLogging = false;
  public static final boolean enableInstanceLogging =
      forceEnableInstanceLogging || Constants.getMode() == Mode.REPLAY;

  public static final double ambiguityThreshold = 1;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double demoTagPosePersistenceSecs = 0.5;
  public static final double coralDetectConfidenceThreshold = 0.35;
  public static final double algaeDetectConfidenceThreshold = 0.35;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

  private static int monoExposure = 2200;
  private static double monoGain = 17.5;

  public static CameraConfig[] cameras =
      new CameraConfig[] {
        CameraConfig.builder()
            .pose(() -> new Pose3d())
            .id("front_cam")
            .width(1600)
            .height(1304)
            .exposure(monoExposure)
            .gain(monoGain)
            .stdDevFactor(1.0)
            .build()
      };

  @Builder
  public record CameraConfig(
      Supplier<Pose3d> pose,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor) {}

  private VisionConstants() {}
}
