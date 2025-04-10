// Copyright (c) Choreo contributors

package frc.robot.util.LoggedChoreo;

import static edu.wpi.first.wpilibj.Alert.AlertType.kWarning;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import choreo.util.ChoreoAlert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.LoggedChoreo.LoggedAutoFactory.AllianceContext;
import java.util.function.BooleanSupplier;

/**
 * An object that represents an autonomous routine.
 *
 * <p>This object is used to handle autonomous trigger logic and schedule commands for a single
 * autonomous routine. This object should **not** be shared across multiple autonomous routines.
 *
 * @see AutoFactory#newRoutine Creating a routine from a AutoFactory
 */
public class LoggedAutoRoutine {
  /**
   * The factory that created this loop. This is used to create commands that are associated with
   * this loop.
   */
  private final LoggedAutoFactory factory;

  /** The underlying {@link EventLoop} that triggers are bound to and polled */
  private final EventLoop loop = new EventLoop();

  /** The name of the auto routine this loop is associated with */
  private final String name;

  /** The alliance helper that is used to determine flipping logic */
  final LoggedAutoFactory.AllianceContext allianceCtx;

  /** A boolean utilized in {@link #active()} to resolve trueness */
  private boolean isActive = false;

  private final Trigger isActiveTrigger =
      new Trigger(loop, () -> isActive && DriverStation.isEnabled());

  /** A boolean indicating if a trajectory is running on the routine right now */
  private boolean isIdle = true;

  private final Trigger isIdleTrigger = new Trigger(loop, () -> isIdle);

  /** A boolean that is true when the loop is killed */
  boolean isKilled = false;

  /** The amount of times the routine has been polled */
  private int pollCount = 0;

  /** The timestamp of the current cycle */
  private double cycleTimestamp = 0;

  /**
   * Creates a new loop with a specific name and a custom alliance supplier.
   *
   * @param factory The factory that created this loop
   * @param name The name of the loop
   * @param allianceHelper The alliance helper that is used to determine flipping logic
   * @see AutoFactory#newRoutine Creating a loop from a AutoFactory
   */
  LoggedAutoRoutine(LoggedAutoFactory factory, String name, AllianceContext allianceHelper) {
    this.factory = (LoggedAutoFactory) factory;
    this.name = name;
    this.allianceCtx = allianceHelper;
  }

  /**
   * Returns a {@link Trigger} that is true while this autonomous routine is being polled.
   *
   * <p>Using a {@link Trigger#onFalse(Command)} will do nothing as when this is false the routine
   * is not being polled anymore.
   *
   * @return A {@link Trigger} that is true while this autonomous routine is being polled.
   */
  public Trigger active() {
    InternalLoggedTracer.reset();
    InternalLoggedTracer.record("Active", "Choreo/LoggedAutoRoutine/Active");
    return isActiveTrigger;
  }

  /** Polls the routine. Should be called in the autonomous periodic method. */
  public void poll() {
    InternalLoggedTracer.reset();
    if (DriverStation.isDisabled() || !allianceCtx.allianceKnownOrIgnored() || isKilled) {
      isActive = false;
      InternalLoggedTracer.record("Poll", "Choreo/LoggedAutoRoutine/Poll");
      return;
    }
    pollCount++;
    cycleTimestamp = Timer.getFPGATimestamp();
    loop.poll();
    isActive = true;
    InternalLoggedTracer.record("Poll", "Choreo/LoggedAutoRoutine/Poll");
  }

  /**
   * Gets the event loop that this routine is using.
   *
   * @return The event loop that this routine is using.
   */
  public EventLoop loop() {
    InternalLoggedTracer.reset();
    InternalLoggedTracer.record("Loop", "Choreo/LoggedAutoRoutine/Loop");
    return loop;
  }

  /**
   * Creates a {@link Trigger} that is bound to the routine's {@link EventLoop}.
   *
   * @param condition The condition represented by the trigger.
   * @return A {@link Trigger} that mirrors the state of the provided {@code condition}
   */
  public Trigger observe(BooleanSupplier condition) {
    InternalLoggedTracer.reset();
    Trigger trigger = new Trigger(loop, condition);
    InternalLoggedTracer.record("Observe", "Choreo/LoggedAutoRoutine/Observe");
    return trigger;
  }

  int pollCount() {
    InternalLoggedTracer.reset();
    InternalLoggedTracer.record("PollCount", "Choreo/LoggedAutoRoutine/PollCount");
    return pollCount;
  }

  double cycleTimestamp() {
    InternalLoggedTracer.reset();
    InternalLoggedTracer.record("CycleTimestamp", "Choreo/LoggedAutoRoutine/CycleTimestamp");
    return cycleTimestamp;
  }

  /**
   * Updates the idle state of the routine.
   *
   * @param isIdle The new idle state of the routine.
   */
  void updateIdle(boolean isIdle) {
    InternalLoggedTracer.reset();
    this.isIdle = isIdle;
    InternalLoggedTracer.record("UpdateIdle", "Choreo/LoggedAutoRoutine/UpdateIdle");
  }

  /**
   * Resets the routine. This can either be called on auto init or auto end to reset the routine
   * incase you run it again. If this is called on a routine that doesn't need to be reset it will
   * do nothing.
   */
  public void reset() {
    InternalLoggedTracer.reset();
    pollCount = 0;
    cycleTimestamp = 0;
    isActive = false;
    InternalLoggedTracer.record("Reset", "Choreo/LoggedAutoRoutine/Reset");
  }

  /** Kills the loop and prevents it from running again. */
  public void kill() {
    InternalLoggedTracer.reset();
    CommandScheduler.getInstance().cancelAll();
    if (isKilled) {
      InternalLoggedTracer.record("Kill", "Choreo/LoggedAutoRoutine/Kill");
      return;
    }
    reset();
    ChoreoAlert.alert("Killed an auto loop", kWarning).set(true);
    isKilled = true;
    InternalLoggedTracer.record("Kill", "Choreo/LoggedAutoRoutine/Kill");
  }

  /**
   * Creates a trigger that is true when the routine is idle.
   *
   * <p>Idle is defined as no trajectories made by the routine are running.
   *
   * @return A trigger that is true when the routine is idle.
   */
  public Trigger idle() {
    InternalLoggedTracer.reset();
    InternalLoggedTracer.record("Idle", "Choreo/LoggedAutoRoutine/Idle");
    return isIdleTrigger;
  }

  /**
   * Creates a new {@link LoggedAutoTrajectory} to be used in an auto routine.
   *
   * @param trajectoryName The name of the trajectory to use.
   * @return A new {@link LoggedAutoTrajectory}.
   */
  public LoggedAutoTrajectory trajectory(String trajectoryName) {
    InternalLoggedTracer.reset();
    LoggedAutoTrajectory result = factory.trajectory(trajectoryName, this, true);
    InternalLoggedTracer.record("Trajectory", "Choreo/LoggedAutoRoutine/Trajectory");
    return result;
  }

  /**
   * Creates a new {@link LoggedAutoTrajectory} to be used in an auto routine.
   *
   * @param trajectoryName The name of the trajectory to use.
   * @param splitIndex The index of the split trajectory to use.
   * @return A new {@link LoggedAutoTrajectory}.
   */
  public LoggedAutoTrajectory trajectory(String trajectoryName, final int splitIndex) {
    InternalLoggedTracer.reset();
    LoggedAutoTrajectory result = factory.trajectory(trajectoryName, splitIndex, this, true);
    InternalLoggedTracer.record("TrajectorySplit", "Choreo/LoggedAutoRoutine/TrajectorySplit");
    return result;
  }

  /**
   * Creates a new {@link LoggedAutoTrajectory} to be used in an auto routine.
   *
   * @param <SampleType> The type of the trajectory samples.
   * @param trajectory The trajectory to use.
   * @return A new {@link LoggedAutoTrajectory}.
   */
  public <SampleType extends TrajectorySample<SampleType>> LoggedAutoTrajectory trajectory(
      Trajectory<SampleType> trajectory) {
    InternalLoggedTracer.reset();
    LoggedAutoTrajectory result = factory.trajectory(trajectory, this, true);
    InternalLoggedTracer.record("TrajectoryObject", "Choreo/LoggedAutoRoutine/TrajectoryObject");
    return result;
  }

  /**
   * Creates a trigger that produces a rising edge when any of the trajectories are finished.
   *
   * @param trajectory The first trajectory to watch.
   * @param trajectories The other trajectories to watch
   * @return a trigger that determines if any of the trajectories are finished
   * @see #anyDone(int, LoggedAutoTrajectory, LoggedAutoTrajectory...) A version of this method that
   *     takes a delay in cycles before the trigger is true.
   */
  public Trigger anyDone(LoggedAutoTrajectory trajectory, LoggedAutoTrajectory... trajectories) {
    InternalLoggedTracer.reset();
    Trigger result = anyDone(0, trajectory, trajectories);
    InternalLoggedTracer.record("AnyDone", "Choreo/LoggedAutoRoutine/AnyDone");
    return result;
  }

  /**
   * Creates a trigger that produces a rising edge when any of the trajectories are finished.
   *
   * @param cyclesToDelay The number of cycles to delay.
   * @param trajectory The first trajectory to watch.
   * @param trajectories The other trajectories to watch
   * @return a trigger that goes true for one cycle whenever any of the trajectories finishes,
   *     delayed by the given number of cycles.
   * @see LoggedAutoTrajectory#doneDelayed(int)
   */
  public Trigger anyDoneDelayed(
      int cyclesToDelay, LoggedAutoTrajectory trajectory, LoggedAutoTrajectory... trajectories) {
    InternalLoggedTracer.reset();
    var trigger = trajectory.doneDelayed(cyclesToDelay);
    for (int i = 0; i < trajectories.length; i++) {
      trigger = trigger.or(trajectories[i].doneDelayed(cyclesToDelay));
    }
    InternalLoggedTracer.record("AnyDoneDelayed", "Choreo/LoggedAutoRoutine/AnyDoneDelayed");
    return trigger.and(this.active());
  }

  /**
   * Creates a trigger that produces a rising edge when any of the trajectories are finished.
   *
   * @param cyclesToDelay The number of cycles to delay.
   * @param trajectory The first trajectory to watch.
   * @param trajectories The other trajectories to watch
   * @return a trigger that determines if any of the trajectories are finished
   * @see LoggedAutoTrajectory#doneDelayed(int)
   * @see AutoRoutine#anyDoneDelayed
   * @deprecated This method is deprecated and will be removed in 2025. Use {@link #anyDoneDelayed}
   */
  @Deprecated(forRemoval = true, since = "2025")
  public Trigger anyDone(
      int cyclesToDelay, LoggedAutoTrajectory trajectory, LoggedAutoTrajectory... trajectories) {
    InternalLoggedTracer.reset();
    Trigger result = anyDoneDelayed(cyclesToDelay, trajectory, trajectories);
    InternalLoggedTracer.record("AnyDoneDeprecated", "Choreo/LoggedAutoRoutine/AnyDoneDeprecated");
    return result;
  }

  /**
   * Creates a trigger that returns true when any of the trajectories given are active.
   *
   * @param trajectory The first trajectory to watch.
   * @param trajectories The other trajectories to watch
   * @return a trigger that determines if any of the trajectories are active
   */
  public Trigger anyActive(LoggedAutoTrajectory trajectory, LoggedAutoTrajectory... trajectories) {
    InternalLoggedTracer.reset();
    var trigger = trajectory.active();
    for (int i = 0; i < trajectories.length; i++) {
      trigger = trigger.or(trajectories[i].active());
    }
    InternalLoggedTracer.record("AnyActive", "Choreo/LoggedAutoRoutine/AnyActive");
    return trigger.and(this.active());
  }

  /**
   * Creates a trigger that returns true when any of the trajectories given are inactive.
   *
   * <p>This trigger will only return true if the routine is active.
   *
   * @param trajectory The first trajectory to watch.
   * @param trajectories The other trajectories to watch
   * @return a trigger that determines if any of the trajectories are inactive
   */
  public Trigger allInactive(
      LoggedAutoTrajectory trajectory, LoggedAutoTrajectory... trajectories) {
    InternalLoggedTracer.reset();
    var trigger = trajectory.inactive();
    for (int i = 0; i < trajectories.length; i++) {
      trigger = trigger.and(trajectories[i].inactive());
    }
    InternalLoggedTracer.record("AllInactive", "Choreo/LoggedAutoRoutine/AllInactive");
    return trigger.and(this.active());
  }

  /**
   * Creates a command that will poll this event loop and reset it when it is cancelled.
   *
   * <p>The command will end instantly and kill the routine if the alliance supplier returns an
   * empty optional when the command is scheduled.
   *
   * @return A command that will poll this event loop and reset it when it is cancelled.
   * @see #cmd(BooleanSupplier) A version of this method that takes a condition to finish the loop.
   */
  public Command cmd() {
    InternalLoggedTracer.reset();
    Command result = cmd(() -> false);
    InternalLoggedTracer.record("Cmd", "Choreo/LoggedAutoRoutine/Cmd");
    return result;
  }

  /**
   * Creates a command that will poll this event loop and reset it when it is finished or canceled.
   *
   * <p>The command will end instantly and kill the routine if the alliance supplier returns an
   * empty optional when the command is scheduled.
   *
   * @param finishCondition A condition that will finish the loop when it is true.
   * @return A command that will poll this event loop and reset it when it is finished or canceled.
   * @see #cmd() A version of this method that doesn't take a condition and never finishes except if
   *     the alliance supplier returns an empty optional when scheduled.
   */
  public Command cmd(BooleanSupplier finishCondition) {
    InternalLoggedTracer.reset();
    Command result =
        Commands.either(
            Commands.run(this::poll)
                .finallyDo(this::reset)
                .until(() -> DriverStation.isDisabled() || finishCondition.getAsBoolean())
                .withName(name),
            Commands.runOnce(
                () -> {
                  ChoreoAlert.alert("Alliance not known when starting routine", kWarning).set(true);
                  kill();
                }),
            allianceCtx::allianceKnownOrIgnored);
    InternalLoggedTracer.record("CmdWithCondition", "Choreo/LoggedAutoRoutine/CmdWithCondition");
    return result;
  }
}
