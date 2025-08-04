package frc.robot.subsystems.shared.vision;

import edu.wpi.first.math.VecBuilder;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.util.InternalLoggedTracer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Camera {
  private final CameraIOInputsAutoLogged inputs;

  private final CameraIO io;
  private final String name;

  public Camera(CameraIO io) {
    inputs = new CameraIOInputsAutoLogged();
    this.io = io;
    this.name = io.getName();
  }

  public void periodic() {
    InternalLoggedTracer.reset();
    io.updateInputs(inputs);
    InternalLoggedTracer.record("Update Inputs", "Vision/Cameras/" + name + "Periodic");

    InternalLoggedTracer.reset();
    Logger.processInputs("Vision/Cameras/" + name, inputs);
    InternalLoggedTracer.record("Process Inputs", "Vision/Cameras/" + name + "Periodic");

    List<VisionObservation> observations = new ArrayList<>();
    for (int i = 0; i < inputs.primaryPose.length; i++) {
      double xyStdDev =
          io.getPrimaryXYStandardDeviationCoefficient()
              * Math.pow(inputs.averageDistance[i], 1.2)
              / Math.pow(inputs.totalTargets[i], 2.0);
      double thetaStdDev =
          inputs.useVisionRotation[i]
              ? io.getThetaStandardDeviationCoefficient()
                  * Math.pow(inputs.averageDistance[i], 1.2)
                  / Math.pow(inputs.totalTargets[i], 2.0)
              : Double.POSITIVE_INFINITY;
      observations.add(
          new VisionObservation(
              inputs.primaryPose[i],
              inputs.frameTimestamp[i],
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
    }

    observations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState::addFieldLocalizerVisionMeasurement);
    observations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState::addReefLocalizerVisionMeasurement);
  }
}
