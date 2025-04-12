// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Utility class for logging code execution times. */
public class InternalLoggedTracer {
  private InternalLoggedTracer() {}

  private static double startTime = -1.0;

  /** Reset the clock. */
  public static void reset() {
    startTime = Timer.getFPGATimestamp();
  }

  /** Save the time elapsed since the last reset or record. */
  public static void record(String epochName, String prefix) {
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput(
        "LoggedTracer/" + prefix + "/" + epochName + "MS", (now - startTime) * 1000.0);
    startTime = now;
  }
}
