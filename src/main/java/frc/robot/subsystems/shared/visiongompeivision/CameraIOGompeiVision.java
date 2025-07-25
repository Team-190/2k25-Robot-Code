// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shared.visiongompeivision;

import static frc.robot.subsystems.shared.visiongompeivision.VisionConstants.*;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.util.SystemTimeValidReader;

public class CameraIOGompeiVision implements CameraIO {
  private final String deviceId;
  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber fpsAprilTagsSubscriber;
  private final StringPublisher eventNamePublisher;
  private final IntegerPublisher matchTypePublisher;
  private final IntegerPublisher matchNumberPublisher;
  private final IntegerPublisher timestampPublisher;
  private final BooleanPublisher isRecordingPublisher;

  private final Timer slowPeriodicTimer = new Timer();

  public CameraIOGompeiVision(int index, String name) {
    this.deviceId = name;
    var gvTable = NetworkTableInstance.getDefault().getTable(this.deviceId);
    var configTable = gvTable.getSubTable("config");
    var camera = cameras[index];

    configTable.getStringTopic("camera_id").publish().set(camera.id());
    configTable.getIntegerTopic("camera_resolution_width").publish().set(camera.width());
    configTable.getIntegerTopic("camera_resolution_height").publish().set(camera.height());
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(camera.autoExposure());
    configTable.getIntegerTopic("camera_exposure").publish().set(camera.exposure());
    configTable.getDoubleTopic("camera_gain").publish().set(camera.gain());
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
    isRecordingPublisher = configTable.getBooleanTopic("is_recording").publish();
    isRecordingPublisher.set(false);
    timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
    eventNamePublisher = configTable.getStringTopic("event_name").publish();
    matchTypePublisher = configTable.getIntegerTopic("match_type").publish();
    matchNumberPublisher = configTable.getIntegerTopic("match_number").publish();

    var outputTable = gvTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    fpsAprilTagsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);

    slowPeriodicTimer.start();
  }

  public void updateInputs(CameraIOInputs inputs, AprilTagCameraIOInputs aprilTagInputs) {
    boolean slowPeriodic = slowPeriodicTimer.advanceIfElapsed(1.0);

    // Update NT connection status
    inputs.ntConnected = false;
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      if (client.remote_id.startsWith(this.deviceId)) {
        inputs.ntConnected = true;
        break;
      }
    }

    // Publish timestamp
    if (slowPeriodic && SystemTimeValidReader.isValid()) {
      timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1000000);
    }

    if (slowPeriodic) {
      eventNamePublisher.set(DriverStation.getEventName());
      matchTypePublisher.set(DriverStation.getMatchType().ordinal());
      matchNumberPublisher.set(DriverStation.getMatchNumber());
    }

    // Get AprilTag data
    var aprilTagQueue = observationSubscriber.readQueue();
    aprilTagInputs.timestamps = new double[aprilTagQueue.length];
    aprilTagInputs.frames = new double[aprilTagQueue.length][];
    for (int i = 0; i < aprilTagQueue.length; i++) {
      aprilTagInputs.timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
      aprilTagInputs.frames[i] = aprilTagQueue[i].value;
    }
    if (slowPeriodic) {
      aprilTagInputs.fps = fpsAprilTagsSubscriber.get();
    }
  }

  public void setRecording(boolean active) {
    isRecordingPublisher.set(active);
  }
}
