// Copyright (c) Choreo contributors

package frc.robot.util.LoggedChoreo;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import choreo.util.ChoreoAlert;
import choreo.util.ChoreoAlert.MultiAlert;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.LoggedChoreo.LoggedAutoFactory.AllianceContext;
import frc.robot.util.LoggedChoreo.LoggedAutoFactory.AutoBindings;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A class that represents a trajectory that can be used in an autonomous routine and have triggers
 * based off of it.
 */
public class LoggedAutoTrajectory {
  // For any devs looking through this class wondering
  // about all the type casting and `?` for generics it's intentional.
  // My goal was to make the sample type minimally leak into user code
  // so you don't have to retype the sample type everywhere in your auto
  // code. This also makes the places with generics exposed to users few
  // and far between. This helps with more novice users

  private static final MultiAlert triggerTimeNegative =
      ChoreoAlert.multiAlert(causes -> "Trigger time cannot be negative for " + causes, kError);
  private static final MultiAlert triggerTimeAboveMax =
      ChoreoAlert.multiAlert(
          causes -> "Trigger time cannot be greater than total trajectory time for " + causes + ".",
          kError);
  private static final MultiAlert eventNotFound =
      ChoreoAlert.multiAlert(causes -> "Event Markers " + causes + " not found.", kError);
  private static final MultiAlert noSamples =
      ChoreoAlert.multiAlert(causes -> "Trajectories " + causes + " have no samples.", kError);
  private static final MultiAlert noInitialPose =
      ChoreoAlert.multiAlert(
          causes -> "Unable to get initial pose for trajectories " + causes + ".", kError);
  private static final Alert allianceNotReady =
      ChoreoAlert.alert("Alliance used but not ready", kError);

  private final String name;
  private final Trajectory<? extends TrajectorySample<?>> trajectory;
  private final TrajectoryLogger<? extends TrajectorySample<?>> trajectoryLogger;
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Pose2d> resetOdometry;
  private final Consumer<? extends TrajectorySample<?>> controller;
  private final AllianceContext allianceCtx;
  private final Timer activeTimer = new Timer();
  private final Timer inactiveTimer = new Timer();
  private final Subsystem driveSubsystem;
  private final LoggedAutoRoutine routine;

  /**
   * A way to create slightly less triggers for many actions. Not static as to not leak triggers
   * made here into another static EventLoop.
   */
  private final Trigger offTrigger;

  /** If this trajectory us currently running */
  private boolean isActive = false;

  /** If the trajectory ran to completion */
  private boolean isCompleted = false;

  /**
   * Constructs an AutoTrajectory.
   *
   * @param name The trajectory name.
   * @param trajectory The trajectory samples.
   * @param poseSupplier The pose supplier.
   * @param controller The controller function.
   * @param allianceCtx The alliance context.
   * @param trajectoryLogger Optional trajectory logger.
   * @param driveSubsystem Drive subsystem.
   * @param routine Event loop.
   * @param bindings {@link AutoFactory}
   */
  public <SampleType extends TrajectorySample<SampleType>> LoggedAutoTrajectory(
      String name,
      Trajectory<SampleType> trajectory,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetOdometry,
      Consumer<SampleType> controller,
      AllianceContext allianceCtx,
      TrajectoryLogger<SampleType> trajectoryLogger,
      Subsystem driveSubsystem,
      LoggedAutoRoutine routine,
      AutoBindings bindings) {
    this.name = name;
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.resetOdometry = resetOdometry;
    this.controller = controller;
    this.allianceCtx = allianceCtx;
    this.driveSubsystem = driveSubsystem;
    this.routine = routine;
    this.offTrigger = new Trigger(routine.loop(), () -> false);
    this.trajectoryLogger = trajectoryLogger;

    bindings.getBindings().forEach((key, value) -> active().and(atTime(key)).onTrue(value));
  }

  @SuppressWarnings("unchecked")
  private void logTrajectory(boolean starting) {
    InternalLoggedTracer.reset();
    var sampleOpt = trajectory.getInitialSample(false);
    if (sampleOpt.isEmpty()) {
      InternalLoggedTracer.record("LogTrajectory", "Choreo/LoggedAutoTrajectory/LogTrajectory");
      return;
    }
    var sample = sampleOpt.get();
    if (sample instanceof SwerveSample) {
      TrajectoryLogger<SwerveSample> swerveLogger =
          (TrajectoryLogger<SwerveSample>) trajectoryLogger;
      Trajectory<SwerveSample> swerveTrajectory = (Trajectory<SwerveSample>) trajectory;
      swerveLogger.accept(swerveTrajectory, starting);
    } else if (sample instanceof DifferentialSample) {
      TrajectoryLogger<DifferentialSample> differentialLogger =
          (TrajectoryLogger<DifferentialSample>) trajectoryLogger;
      Trajectory<DifferentialSample> differentialTrajectory =
          (Trajectory<DifferentialSample>) trajectory;
      differentialLogger.accept(differentialTrajectory, starting);
    }
    InternalLoggedTracer.record("LogTrajectory", "Choreo/LoggedAutoTrajectory/LogTrajectory");
  }

  private void cmdInitialize() {
    InternalLoggedTracer.reset();
    activeTimer.start();
    inactiveTimer.stop();
    inactiveTimer.reset();
    isActive = true;
    isCompleted = false;
    logTrajectory(true);
    routine.updateIdle(false);
    InternalLoggedTracer.record("CmdInitialize", "Choreo/LoggedAutoTrajectory/CmdInitialize");
  }

  @SuppressWarnings("unchecked")
  private void cmdExecute() {
    InternalLoggedTracer.reset();
    if (!allianceCtx.allianceKnownOrIgnored()) {
      allianceNotReady.set(true);
      InternalLoggedTracer.record("CmdExecute", "Choreo/LoggedAutoTrajectory/CmdExecute");
      return;
    }
    var sampleOpt = trajectory.sampleAt(activeTimer.get(), allianceCtx.doFlip());
    if (sampleOpt.isEmpty()) {
      InternalLoggedTracer.record("CmdExecute", "Choreo/LoggedAutoTrajectory/CmdExecute");
      return;
    }
    var sample = sampleOpt.get();
    if (sample instanceof SwerveSample swerveSample) {
      var swerveController = (Consumer<SwerveSample>) this.controller;
      swerveController.accept(swerveSample);
    } else if (sample instanceof DifferentialSample differentialSample) {
      var differentialController = (Consumer<DifferentialSample>) this.controller;
      differentialController.accept(differentialSample);
    }
    InternalLoggedTracer.record("CmdExecute", "Choreo/LoggedAutoTrajectory/CmdExecute");
  }

  @SuppressWarnings("unchecked")
  private void cmdEnd(boolean interrupted) {
    InternalLoggedTracer.reset();
    activeTimer.stop();
    activeTimer.reset();
    inactiveTimer.start();
    isActive = false;
    isCompleted = !interrupted;

    if (!interrupted && allianceCtx.allianceKnownOrIgnored()) {
      var sampleOpt = trajectory.getFinalSample(allianceCtx.doFlip());
      if (sampleOpt.isPresent()) {
        var sample = sampleOpt.get();
        if (sample instanceof SwerveSample swerveSample) {
          var swerveController = (Consumer<SwerveSample>) this.controller;
          swerveController.accept(swerveSample);
        } else if (sample instanceof DifferentialSample differentialSample) {
          var differentialController = (Consumer<DifferentialSample>) this.controller;
          differentialController.accept(differentialSample);
        }
      }
    }

    logTrajectory(false);
    routine.updateIdle(true);
    InternalLoggedTracer.record("CmdEnd", "Choreo/LoggedAutoTrajectory/CmdEnd");
  }

  private boolean cmdIsFinished() {
    InternalLoggedTracer.reset();
    boolean result =
        activeTimer.get() > trajectory.getTotalTime()
            || !routine.active().getAsBoolean()
            || !allianceCtx.allianceKnownOrIgnored();
    InternalLoggedTracer.record("CmdIsFinished", "Choreo/LoggedAutoTrajectory/CmdIsFinished");
    return result;
  }

  /**
   * Creates a command that allocates the drive subsystem and follows the trajectory using the
   * factories control function
   *
   * @return The command that will follow the trajectory
   */
  public Command cmd() {
    InternalLoggedTracer.reset();
    Command result;
    if (trajectory.samples().isEmpty()) {
      result =
          driveSubsystem.runOnce(() -> noSamples.addCause(name)).withName("Trajectory_" + name);
    } else {
      result =
          new FunctionalCommand(
                  this::cmdInitialize,
                  this::cmdExecute,
                  this::cmdEnd,
                  this::cmdIsFinished,
                  driveSubsystem)
              .withName("Trajectory_" + name);
    }
    InternalLoggedTracer.record("Cmd", "Choreo/LoggedAutoTrajectory/Cmd");
    return result;
  }

  /**
   * Creates a command that will schedule <b>another</b> command that will follow the trajectory.
   *
   * <p>This can be useful when putting {@link AutoTrajectory} commands in sequences that require
   * subsystems also required by in AutoTrajectory-bound subsystems.
   *
   * @return The command that will schedule the trajectory following command.
   */
  public Command spawnCmd() {
    InternalLoggedTracer.reset();
    Command result = new ScheduleCommand(cmd()).withName("Trajectory_" + name + "_Spawner");
    InternalLoggedTracer.record("SpawnCmd", "Choreo/LoggedAutoTrajectory/SpawnCmd");
    return result;
  }

  /**
   * Creates a command that resets the robot's odometry to the start of this trajectory.
   *
   * @return A command that resets the robot's odometry.
   */
  public Command resetOdometry() {
    InternalLoggedTracer.reset();
    Command result =
        Commands.either(
                Commands.runOnce(
                    () -> resetOdometry.accept(getInitialPose().get()), driveSubsystem),
                Commands.runOnce(
                        () -> {
                          noInitialPose.addCause(name);
                          routine.kill();
                        })
                    .andThen(driveSubsystem.run(() -> {})),
                () -> getInitialPose().isPresent())
            .withName("Trajectory_ResetOdometry_" + name);
    InternalLoggedTracer.record("ResetOdometry", "Choreo/LoggedAutoTrajectory/ResetOdometry");
    return result;
  }

  /**
   * Will get the underlying {@link Trajectory} object.
   *
   * <p><b>WARNING:</b> This method is not type safe and should be used with caution. The sample
   * type of the trajectory should be known before calling this method.
   *
   * @param <SampleType> The type of the trajectory samples.
   * @return The underlying {@link Trajectory} object.
   */
  @SuppressWarnings("unchecked")
  public <SampleType extends TrajectorySample<SampleType>>
      Trajectory<SampleType> getRawTrajectory() {
    InternalLoggedTracer.reset();
    Trajectory<SampleType> result = (Trajectory<SampleType>) trajectory;
    InternalLoggedTracer.record("GetRawTrajectory", "Choreo/LoggedAutoTrajectory/GetRawTrajectory");
    return result;
  }

  /**
   * Will get the starting pose of the trajectory.
   *
   * <p>This position is flipped if alliance flipping is enabled and the alliance supplier returns
   * Red.
   *
   * <p>This method returns an empty Optional if the trajectory is empty. This method returns an
   * empty Optional if alliance flipping is enabled and the alliance supplier returns an empty
   * Optional.
   *
   * @return The starting pose
   */
  public Optional<Pose2d> getInitialPose() {
    InternalLoggedTracer.reset();
    Optional<Pose2d> result;
    if (!allianceCtx.allianceKnownOrIgnored()) {
      allianceNotReady.set(true);
      result = Optional.empty();
    } else {
      result = trajectory.getInitialPose(allianceCtx.doFlip());
    }
    InternalLoggedTracer.record("GetInitialPose", "Choreo/LoggedAutoTrajectory/GetInitialPose");
    return result;
  }

  /**
   * Will get the ending pose of the trajectory.
   *
   * <p>This position is flipped if alliance flipping is enabled and the alliance supplier returns
   * Red.
   *
   * <p>This method returns an empty Optional if the trajectory is empty. This method returns an
   * empty Optional if alliance flipping is enabled and the alliance supplier returns an empty
   * Optional.
   *
   * @return The starting pose
   */
  public Optional<Pose2d> getFinalPose() {
    InternalLoggedTracer.reset();
    Optional<Pose2d> result;
    if (!allianceCtx.allianceKnownOrIgnored()) {
      allianceNotReady.set(true);
      result = Optional.empty();
    } else {
      result = trajectory.getFinalPose(allianceCtx.doFlip());
    }
    InternalLoggedTracer.record("GetFinalPose", "Choreo/LoggedAutoTrajectory/GetFinalPose");
    return result;
  }

  /**
   * Returns a trigger that is true while the trajectory is scheduled.
   *
   * @return A trigger that is true while the trajectory is scheduled.
   */
  public Trigger active() {
    InternalLoggedTracer.reset();
    Trigger result =
        new Trigger(routine.loop(), () -> this.isActive && routine.active().getAsBoolean());
    InternalLoggedTracer.record("Active", "Choreo/LoggedAutoTrajectory/Active");
    return result;
  }

  /**
   * Returns a trigger that is true while the command is not scheduled.
   *
   * <p>The same as calling <code>active().negate()</code>.
   *
   * @return A trigger that is true while the command is not scheduled.
   */
  public Trigger inactive() {
    InternalLoggedTracer.reset();
    Trigger result = active().negate();
    InternalLoggedTracer.record("Inactive", "Choreo/LoggedAutoTrajectory/Inactive");
    return result;
  }

  private Trigger timeTrigger(double targetTime, Timer timer) {
    InternalLoggedTracer.reset();
    Trigger result =
        new Trigger(
            routine.loop(),
            new BooleanSupplier() {
              double lastTimestamp = -1.0;
              OptionalInt pollTarget = OptionalInt.empty();

              public boolean getAsBoolean() {
                if (!timer.isRunning()) {
                  lastTimestamp = -1.0;
                  pollTarget = OptionalInt.empty();
                  return false;
                }
                double nowTimestamp = timer.get();
                try {
                  boolean timeAligns = lastTimestamp < targetTime && nowTimestamp >= targetTime;
                  if (pollTarget.isEmpty() && timeAligns) {
                    // if the time aligns for this cycle and it hasn't aligned previously this cycle
                    pollTarget = OptionalInt.of(routine.pollCount());
                    return true;
                  } else if (pollTarget.isPresent()
                      && routine.pollCount() == pollTarget.getAsInt()) {
                    // if the time aligned previously this cycle
                    return true;
                  } else if (pollTarget.isPresent()) {
                    // if the time aligned last cycle
                    pollTarget = OptionalInt.empty();
                    return false;
                  }
                  return false;
                } finally {
                  lastTimestamp = nowTimestamp;
                }
              }
            });
    InternalLoggedTracer.record("TimeTrigger", "Choreo/LoggedAutoTrajectory/TimeTrigger");
    return result;
  }

  private Trigger enterExitTrigger(Trigger enter, Trigger exit) {
    InternalLoggedTracer.reset();
    Trigger result =
        new Trigger(
            routine.loop(),
            new BooleanSupplier() {
              boolean output = false;

              @Override
              public boolean getAsBoolean() {
                if (enter.getAsBoolean()) {
                  output = true;
                }
                if (exit.getAsBoolean()) {
                  output = false;
                }
                return output;
              }
            });
    InternalLoggedTracer.record("EnterExitTrigger", "Choreo/LoggedAutoTrajectory/EnterExitTrigger");
    return result;
  }

  /**
   * Returns a trigger that rises to true a number of cycles after the trajectory ends and falls
   * after one pulse.
   *
   * <p>This is different from inactive() in a few ways.
   *
   * <ul>
   *   <li>This will never be true if the trajectory is interrupted
   *   <li>This will never be true before the trajectory is run
   *   <li>This will fall the next cycle after the trajectory ends
   * </ul>
   *
   * <p>Why does the trigger need to fall?
   *
   * <pre><code>
   * //Lets say we had this code segment
   * Trigger hasGamepiece = ...;
   * Trigger noGamepiece = hasGamepiece.negate();
   *
   * AutoTrajectory rushMidTraj = ...;
   * AutoTrajectory goShootGamepiece = ...;
   * AutoTrajectory pickupAnotherGamepiece = ...;
   *
   * routine.enabled().onTrue(rushMidTraj.cmd());
   *
   * rushMidTraj.doneDelayed(10).and(noGamepiece).onTrue(pickupAnotherGamepiece.cmd());
   * rushMidTraj.doneDelayed(10).and(hasGamepiece).onTrue(goShootGamepiece.cmd());
   *
   * // If done never falls when a new trajectory is scheduled
   * // then these triggers leak into the next trajectory, causing the next note pickup
   * // to trigger goShootGamepiece.cmd() even if we no longer care about these checks
   * </code></pre>
   *
   * @param seconds The seconds to delay the trigger from rising to true.
   * @return A trigger that is true when the trajectory is finished.
   */
  public Trigger doneDelayed(double seconds) {
    InternalLoggedTracer.reset();
    Trigger result =
        timeTrigger(seconds, inactiveTimer).and(new Trigger(routine.loop(), () -> isCompleted));
    InternalLoggedTracer.record("DoneDelayed", "Choreo/LoggedAutoTrajectory/DoneDelayed");
    return result;
  }

  /**
   * Returns a trigger that rises to true a number of cycles after the trajectory ends and falls
   * after one pulse.
   *
   * @param cycles The number of cycles to delay the trigger from rising to true.
   * @return A trigger that is true when the trajectory is finished.
   * @see #doneDelayed(int)
   * @deprecated Use {@link #doneDelayed(int)} instead.
   */
  @Deprecated(forRemoval = true, since = "2025")
  public Trigger done(int cycles) {
    InternalLoggedTracer.reset();
    Trigger result = doneDelayed(0.02 * cycles);
    InternalLoggedTracer.record("Done", "Choreo/LoggedAutoTrajectory/Done");
    return result;
  }

  /**
   * Returns a trigger that rises to true when the trajectory ends and falls after one pulse.
   *
   * <p>This is different from inactive() in a few ways.
   *
   * <ul>
   *   <li>This will never be true if the trajectory is interrupted
   *   <li>This will never be true before the trajectory is run
   *   <li>This will fall the next cycle after the trajectory ends
   * </ul>
   *
   * <p>Why does the trigger need to fall?
   *
   * <pre><code>
   * //Lets say we had this code segment
   * Trigger hasGamepiece = ...;
   * Trigger noGamepiece = hasGamepiece.negate();
   *
   * AutoTrajectory rushMidTraj = ...;
   * AutoTrajectory goShootGamepiece = ...;
   * AutoTrajectory pickupAnotherGamepiece = ...;
   *
   * routine.enabled().onTrue(rushMidTraj.cmd());
   *
   * rushMidTraj.done().and(noGamepiece).onTrue(pickupAnotherGamepiece.cmd());
   * rushMidTraj.done().and(hasGamepiece).onTrue(goShootGamepiece.cmd());
   *
   * // If done never falls when a new trajectory is scheduled
   * // then these triggers leak into the next trajectory, causing the next note pickup
   * // to trigger goShootGamepiece.cmd() even if we no longer care about these checks
   * </code></pre>
   *
   * @return A trigger that is true when the trajectory is finished.
   */
  public Trigger done() {
    InternalLoggedTracer.reset();
    Trigger result = doneDelayed(0);
    InternalLoggedTracer.record("Done", "Choreo/LoggedAutoTrajectory/Done");
    return result;
  }

  /**
   * Returns a trigger that stays true for a number of cycles after the trajectory ends.
   *
   * @param seconds Seconds to stay true after the trajectory ends.
   * @return A trigger that stays true for a number of cycles after the trajectory ends.
   */
  public Trigger doneFor(double seconds) {
    InternalLoggedTracer.reset();
    Trigger result = enterExitTrigger(doneDelayed(0), doneDelayed(seconds));
    InternalLoggedTracer.record("DoneFor", "Choreo/LoggedAutoTrajectory/DoneFor");
    return result;
  }

  /**
   * Returns a trigger that is true when the trajectory was the last one active and is done.
   *
   * @return A trigger that is true when the trajectory was the last one active and is done.
   */
  public Trigger recentlyDone() {
    InternalLoggedTracer.reset();
    Trigger result = enterExitTrigger(doneDelayed(0), routine.idle().negate());
    InternalLoggedTracer.record("RecentlyDone", "Choreo/LoggedAutoTrajectory/RecentlyDone");
    return result;
  }

  /**
   * A shorthand for `.done().onTrue(otherTrajectory.cmd())`
   *
   * @param otherTrajectory The other trajectory to run when this one is done.
   */
  public void chain(AutoTrajectory otherTrajectory) {
    InternalLoggedTracer.reset();
    done().onTrue(otherTrajectory.cmd());
    InternalLoggedTracer.record("Chain", "Choreo/LoggedAutoTrajectory/Chain");
  }

  /**
   * Returns a trigger that will go true for 1 cycle when the desired time has elapsed
   *
   * @param timeSinceStart The time since the command started in seconds.
   * @return A trigger that is true when timeSinceStart has elapsed.
   */
  public Trigger atTime(double timeSinceStart) {
    InternalLoggedTracer.reset();
    Trigger result;
    if (timeSinceStart < 0) {
      triggerTimeNegative.addCause(name);
      result = offTrigger;
    } else if (timeSinceStart > trajectory.getTotalTime()) {
      triggerTimeAboveMax.addCause(name);
      result = offTrigger;
    } else {
      result = timeTrigger(timeSinceStart, activeTimer);
    }
    InternalLoggedTracer.record("AtTime", "Choreo/LoggedAutoTrajectory/AtTime");
    return result;
  }

  /**
   * Returns a trigger that will go true for 1 cycle when the desired before the end of the
   * trajectory time.
   *
   * @param timeBeforeEnd The time before the end of the trajectory.
   * @return A trigger that is true when timeBeforeEnd has elapsed.
   */
  public Trigger atTimeBeforeEnd(double timeBeforeEnd) {
    InternalLoggedTracer.reset();
    Trigger result = atTime(trajectory.getTotalTime() - timeBeforeEnd);
    InternalLoggedTracer.record("AtTimeBeforeEnd", "Choreo/LoggedAutoTrajectory/AtTimeBeforeEnd");
    return result;
  }

  /**
   * Returns a trigger that is true when the event with the given name has been reached based on
   * time.
   *
   * <p>A warning will be printed to the DriverStation if the event is not found and the trigger
   * will always be false.
   *
   * @param eventName The name of the event.
   * @return A trigger that is true when the event with the given name has been reached based on
   *     time.
   * @see <a href="https://choreo.autos/usage/editing-paths/#event-markers">Event Markers in the
   *     GUI</a>
   */
  public Trigger atTime(String eventName) {
    InternalLoggedTracer.reset();
    boolean foundEvent = false;
    Trigger result = offTrigger;

    for (var event : trajectory.getEvents(eventName)) {
      result = result.or(atTime(event.timestamp));
      foundEvent = true;
    }

    // The user probably expects an event to exist if they're trying to do something at that event,
    // report the missing event.
    if (!foundEvent) {
      eventNotFound.addCause(name);
    }
    InternalLoggedTracer.record("AtTimeEvent", "Choreo/LoggedAutoTrajectory/AtTimeEvent");
    return result;
  }

  private boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
    InternalLoggedTracer.reset();
    boolean result;
    if (Math.abs(toleranceRadians) > Math.PI) {
      result = true;
    } else {
      double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
      result = dot > Math.cos(toleranceRadians);
    }
    InternalLoggedTracer.record("WithinTolerance", "Choreo/LoggedAutoTrajectory/WithinTolerance");
    return result;
  }

  /**
   * Returns a trigger that is true when the robot is within toleranceMeters of the given pose.
   *
   * <p>The pose is flipped if alliance flipping is enabled and the alliance supplier returns Red.
   *
   * <p>While alliance flipping is enabled and the alliance supplier returns empty, the trigger will
   * return false.
   *
   * @param pose The pose to check against, unflipped.
   * @param toleranceMeters The tolerance in meters.
   * @param toleranceRadians The heading tolerance in radians.
   * @return A trigger that is true when the robot is within toleranceMeters of the given pose.
   */
  public Trigger atPose(Pose2d pose, double toleranceMeters, double toleranceRadians) {
    InternalLoggedTracer.reset();
    Pose2d flippedPose = ChoreoAllianceFlipUtil.flip(pose);
    Trigger result =
        new Trigger(
                routine.loop(),
                () -> {
                  if (allianceCtx.allianceKnownOrIgnored()) {
                    final Pose2d currentPose = poseSupplier.get();
                    if (allianceCtx.doFlip()) {
                      boolean transValid =
                          currentPose.getTranslation().getDistance(flippedPose.getTranslation())
                              < toleranceMeters;
                      boolean rotValid =
                          withinTolerance(
                              currentPose.getRotation(),
                              flippedPose.getRotation(),
                              toleranceRadians);
                      return transValid && rotValid;
                    } else {
                      boolean transValid =
                          currentPose.getTranslation().getDistance(pose.getTranslation())
                              < toleranceMeters;
                      boolean rotValid =
                          withinTolerance(
                              currentPose.getRotation(), pose.getRotation(), toleranceRadians);
                      return transValid && rotValid;
                    }
                  } else {
                    allianceNotReady.set(true);
                    return false;
                  }
                })
            .and(active());
    InternalLoggedTracer.record("AtPose", "Choreo/LoggedAutoTrajectory/AtPose");
    return result;
  }

  /**
   * Returns a trigger that is true when the robot is within toleranceMeters and toleranceRadians of
   * the given event's pose.
   *
   * <p>A warning will be printed to the DriverStation if the event is not found and the trigger
   * will always be false.
   *
   * @param eventName The name of the event.
   * @param toleranceMeters The tolerance in meters.
   * @param toleranceRadians The heading tolerance in radians.
   * @return A trigger that is true when the robot is within toleranceMeters of the given events
   *     pose.
   * @see <a href="https://choreo.autos/usage/editing-paths/#event-markers">Event Markers in the
   *     GUI</a>
   */
  public Trigger atPose(String eventName, double toleranceMeters, double toleranceRadians) {
    InternalLoggedTracer.reset();
    boolean foundEvent = false;
    Trigger result = offTrigger;

    for (var event : trajectory.getEvents(eventName)) {
      // This could create a lot of objects, could be done a more efficient way
      // with having it all be 1 trigger that just has a list of possess and checks each one each
      // cycle or something like that.
      // If choreo starts showing memory issues we can look into this.
      Optional<Pose2d> poseOpt =
          trajectory
              // don't mirror here because the poses are mirrored themselves
              // this also lets atPose be called before the alliance is ready
              .sampleAt(event.timestamp, false)
              .map(TrajectorySample::getPose);
      if (poseOpt.isPresent()) {
        result = result.or(atPose(poseOpt.get(), toleranceMeters, toleranceRadians));
        foundEvent = true;
      }
    }

    // The user probably expects an event to exist if they're trying to do something at that event,
    // report the missing event.
    if (!foundEvent) {
      eventNotFound.addCause(name);
    }
    InternalLoggedTracer.record("AtPoseEvent", "Choreo/LoggedAutoTrajectory/AtPoseEvent");
    return result;
  }

  /**
   * Returns a trigger that is true when the robot is within toleranceMeters of the given
   * translation.
   *
   * <p>The translation is flipped if alliance flipping is enabled and the alliance supplier returns
   * Red.
   *
   * <p>While alliance flipping is enabled and the alliance supplier returns empty, the trigger will
   * return false.
   *
   * @param translation The translation to check against, unflipped.
   * @param toleranceMeters The tolerance in meters.
   * @return A trigger that is true when the robot is within toleranceMeters of the given
   *     translation.
   */
  public Trigger atTranslation(Translation2d translation, double toleranceMeters) {
    InternalLoggedTracer.reset();
    Translation2d flippedTranslation = ChoreoAllianceFlipUtil.flip(translation);
    Trigger result =
        new Trigger(
                routine.loop(),
                () -> {
                  if (allianceCtx.allianceKnownOrIgnored()) {
                    final Translation2d currentTrans = poseSupplier.get().getTranslation();
                    if (allianceCtx.doFlip()) {
                      return currentTrans.getDistance(flippedTranslation) < toleranceMeters;
                    } else {
                      return currentTrans.getDistance(translation) < toleranceMeters;
                    }
                  } else {
                    allianceNotReady.set(true);
                    return false;
                  }
                })
            .and(active());
    InternalLoggedTracer.record("AtTranslation", "Choreo/LoggedAutoTrajectory/AtTranslation");
    return result;
  }

  /**
   * Returns a trigger that is true when the robot is within toleranceMeters and toleranceRadians of
   * the given event's translation.
   *
   * <p>A warning will be printed to the DriverStation if the event is not found and the trigger
   * will always be false.
   *
   * @param eventName The name of the event.
   * @param toleranceMeters The tolerance in meters.
   * @return A trigger that is true when the robot is within toleranceMeters of the given events
   *     translation.
   * @see <a href="https://choreo.autos/usage/editing-paths/#event-markers">Event Markers in the
   *     GUI</a>
   */
  public Trigger atTranslation(String eventName, double toleranceMeters) {
    InternalLoggedTracer.reset();
    boolean foundEvent = false;
    Trigger trig = offTrigger;

    for (var event : trajectory.getEvents(eventName)) {
      // This could create a lot of objects, could be done a more efficient way
      // with having it all be 1 trigger that just has a list of poses and checks each one each
      // cycle or something like that.
      // If choreo starts showing memory issues we can look into this.
      Optional<Translation2d> translationOpt =
          trajectory
              // don't mirror here because the translations are mirrored themselves
              // this also lets atTranslation be called before the alliance is ready
              .sampleAt(event.timestamp, false)
              .map(TrajectorySample::getPose)
              .map(Pose2d::getTranslation);
      if (translationOpt.isPresent()) {
        trig = trig.or(atTranslation(translationOpt.get(), toleranceMeters));
        foundEvent = true;
      }
    }

    // The user probably expects an event to exist if they're trying to do something at that event,
    // report the missing event.
    if (!foundEvent) {
      eventNotFound.addCause(name);
    }
    InternalLoggedTracer.record(
        "AtTranslationEvent", "Choreo/LoggedAutoTrajectory/AtTranslationEvent");
    return trig;
  }

  /**
   * Returns an array of all the timestamps of the events with the given name.
   *
   * @param eventName The name of the event.
   * @return An array of all the timestamps of the events with the given name.
   * @see <a href="https://choreo.autos/usage/editing-paths/#event-markers">Event Markers in the
   *     GUI</a>
   */
  public double[] collectEventTimes(String eventName) {
    InternalLoggedTracer.reset();
    double[] times =
        trajectory.getEvents(eventName).stream()
            .filter(e -> e.timestamp >= 0 && e.timestamp <= trajectory.getTotalTime())
            .mapToDouble(e -> e.timestamp)
            .toArray();

    if (times.length == 0) {
      eventNotFound.addCause("collectEvents(" + eventName + ")");
    }
    InternalLoggedTracer.record(
        "CollectEventTimes", "Choreo/LoggedAutoTrajectory/CollectEventTimes");
    return times;
  }

  /**
   * Returns an array of all the poses of the events with the given name.
   *
   * <p>The returned poses are always unflipped. If you use them in a trigger like `atPose` or
   * `atTranslation`, the library will automatically flip them if necessary. If you intend using
   * them in a different context, you can use {@link ChoreoAllianceFlipUtil#flip} to flip them.
   *
   * @param eventName The name of the event.
   * @return An array of all the poses of the events with the given name.
   * @see <a href="https://choreo.autos/usage/editing-paths/#event-markers">Event Markers in the
   *     GUI</a>
   */
  public Pose2d[] collectEventPoses(String eventName) {
    InternalLoggedTracer.reset();
    double[] times = collectEventTimes(eventName);
    Pose2d[] poses = new Pose2d[times.length];
    for (int i = 0; i < times.length; i++) {
      Pose2d pose =
          trajectory
              .sampleAt(times[i], false)
              .map(TrajectorySample::getPose)
              .get(); // the event times are guaranteed to be valid
      poses[i] = pose;
    }
    InternalLoggedTracer.record(
        "CollectEventPoses", "Choreo/LoggedAutoTrajectory/CollectEventPoses");
    return poses;
  }

  @Override
  public boolean equals(Object obj) {
    return obj instanceof LoggedAutoTrajectory traj && name.equals(traj.name);
  }

  public LoggedAutoTrajectory bindEvent(String eventName, Command command) {
    this.atTime(eventName).onTrue(command);
    return this;
  }
}
