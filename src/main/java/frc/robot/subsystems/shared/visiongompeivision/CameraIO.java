// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shared.visiongompeivision;

import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
  @AutoLog
  class CameraIOInputs {
    public boolean ntConnected = false;
  }

  @AutoLog
  class AprilTagCameraIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;
  }

  default void updateInputs(CameraIOInputs inputs, AprilTagCameraIOInputs aprilTagInputs) {}

  default void setRecording(boolean active) {}
}
