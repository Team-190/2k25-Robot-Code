// Copyright (c) Choreo contributors

package frc.robot.util.LoggedChoreo;

import choreo.Choreo;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A factory used to create {@link AutoRoutine}s and {@link AutoTrajectory}s.
 *
 * @see <a href="https://choreo.autos/choreolib/auto-routines">Auto Routine Docs</a>
 */
public class LoggedAutoFactory {
  static record AllianceContext(
      boolean useAllianceFlipping, Supplier<Optional<Alliance>> allianceGetter) {
    boolean allianceKnownOrIgnored() {
      return allianceGetter.get().isPresent() || !useAllianceFlipping;
    }

    boolean doFlip() {
      return useAllianceFlipping
          && allianceGetter
              .get()
              .orElseThrow(
                  () -> new RuntimeException("Flip check was called with an unknown alliance"))
              .equals(Alliance.Red);
    }

    Optional<Alliance> alliance() {
      return allianceGetter.get();
    }
  }

  /** A class used to bind commands to events in all trajectories created by this factory. */
  static class AutoBindings {
    private HashMap<String, Command> bindings = new HashMap<>();

    /** Default constructor. */
    public AutoBindings() {}

    /**
     * Binds a command to an event in all trajectories created by the factory using this bindings.
     *
     * @param name The name of the event to bind the command to.
     * @param cmd The command to bind to the event.
     * @return The bindings object for chaining.
     */
    public AutoBindings bind(String name, Command cmd) {
      bindings.put(name, cmd);
      return this;
    }

    /**
     * Gets the bindings map.
     *
     * @return The bindings map.
     */
    HashMap<String, Command> getBindings() {
      return bindings;
    }
  }

  private final LoggedTrajectoryCache trajectoryCache = new LoggedTrajectoryCache();
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Pose2d> resetOdometry;
  private final Consumer<? extends TrajectorySample<?>> controller;
  private final AllianceContext allianceCtx;
  private final Subsystem driveSubsystem;
  private final AutoBindings bindings = new AutoBindings();
  private final TrajectoryLogger<? extends TrajectorySample<?>> trajectoryLogger;
  private final LoggedAutoRoutine voidRoutine;

  /**
   * Create a factory that can be used to create {@link AutoRoutine} and {@link AutoTrajectory}.
   *
   * @param <SampleType> The type of samples in the trajectory.
   * @param poseSupplier A function that returns the current field-relative {@link Pose2d} of the
   *     robot.
   * @param resetOdometry A function that receives a field-relative {@link Pose2d} to reset the
   *     robot's odometry to.
   * @param controller A function that receives the current {@link SampleType} and controls the
   *     robot.
   * @param driveSubsystem The drive {@link Subsystem} to require for {@link AutoTrajectory} {@link
   *     Command}s.
   * @param useAllianceFlipping If this is true, when on the red alliance, the path will be mirrored
   *     to the opposite side, while keeping the same coordinate system origin.
   * @param trajectoryLogger A {@link TrajectoryLogger} to log {@link Trajectory} as they start and
   *     finish.
   * @see AutoChooser using this factory with AutoChooser to generate auto routines.
   */
  public <SampleType extends TrajectorySample<SampleType>> LoggedAutoFactory(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetOdometry,
      Consumer<SampleType> controller,
      boolean useAllianceFlipping,
      Subsystem driveSubsystem,
      TrajectoryLogger<SampleType> trajectoryLogger) {
    this.poseSupplier = poseSupplier;
    this.resetOdometry = resetOdometry;
    this.controller = controller;
    this.driveSubsystem = driveSubsystem;
    this.allianceCtx = new AllianceContext(useAllianceFlipping, DriverStation::getAlliance);
    this.trajectoryLogger = trajectoryLogger;
    HAL.report(tResourceType.kResourceType_ChoreoTrigger, 1);

    voidRoutine =
        new LoggedAutoRoutine(this, "VOID-ROUTINE", allianceCtx) {
          @Override
          public Command cmd() {
            return Commands.none().withName("VoidAutoRoutine");
          }

          @Override
          public Command cmd(BooleanSupplier _finishCondition) {
            return cmd();
          }

          @Override
          public void poll() {}

          @Override
          public void reset() {}

          @Override
          public Trigger active() {
            return new Trigger(this.loop(), () -> true);
          }
        };
  }

  /**
   * Create a factory that can be used to create {@link AutoRoutine} and {@link AutoTrajectory}.
   *
   * @param <ST> {@link choreo.trajectory.DifferentialSample} or {@link
   *     choreo.trajectory.SwerveSample}
   * @param poseSupplier A function that returns the current field-relative {@link Pose2d} of the
   *     robot.
   * @param resetOdometry A function that receives a field-relative {@link Pose2d} to reset the
   *     robot's odometry to.
   * @param controller A function that receives the current {@link ST} and controls the robot.
   * @param driveSubsystem The drive {@link Subsystem} to require for {@link AutoTrajectory} {@link
   *     Command}s.
   * @param useAllianceFlipping If this returns true, when on the red alliance, the path will be
   *     mirrored to the opposite side, while keeping the same coordinate system origin.
   * @see AutoChooser using this factory with AutoChooser to generate auto routines.
   */
  public <ST extends TrajectorySample<ST>> LoggedAutoFactory(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetOdometry,
      Consumer<ST> controller,
      boolean useAllianceFlipping,
      Subsystem driveSubsystem) {
    this(
        poseSupplier,
        resetOdometry,
        controller,
        useAllianceFlipping,
        driveSubsystem,
        (sample, isStart) -> {});
  }

  /**
   * Creates a new {@link AutoRoutine}.
   *
   * @param name The name of the {@link AutoRoutine}.
   * @return A new {@link AutoRoutine}.
   */
  public LoggedAutoRoutine newRoutine(String name) {
    // Clear cache in simulation to allow a form of "hot-reloading" trajectories
    // if (RobotBase.isSimulation()) {
    //   trajectoryCache.clear();
    // }

    return new LoggedAutoRoutine(this, name, allianceCtx);
  }

  /**
   * A package protected method to create a new {@link AutoTrajectory} to be used in an {@link
   * AutoRoutine}.
   *
   * @see AutoRoutine#trajectory(String)
   */
  public LoggedAutoTrajectory trajectory(
      String trajectoryName, LoggedAutoRoutine routine, boolean useBindings) {
    Optional<? extends Trajectory<?>> optTrajectory =
        trajectoryCache.loadTrajectory(trajectoryName);
    Trajectory<?> trajectory;
    if (optTrajectory.isPresent()) {
      trajectory = optTrajectory.get();
    } else {
      trajectory = new Trajectory<SwerveSample>(trajectoryName, List.of(), List.of(), List.of());
    }
    return trajectory(trajectory, routine, useBindings);
  }

  /**
   * A package protected method to create a new {@link AutoTrajectory} to be used in an {@link
   * AutoRoutine}.
   *
   * @see AutoRoutine#trajectory(String, int)
   */
  public LoggedAutoTrajectory trajectory(
      String trajectoryName, final int splitIndex, LoggedAutoRoutine routine, boolean useBindings) {
    Optional<? extends Trajectory<?>> optTrajectory =
        trajectoryCache.loadTrajectory(trajectoryName, splitIndex);
    Trajectory<?> trajectory;
    if (optTrajectory.isPresent()) {
      trajectory = optTrajectory.get();
    } else {
      trajectory = new Trajectory<SwerveSample>(trajectoryName, List.of(), List.of(), List.of());
    }
    return trajectory(trajectory, routine, useBindings);
  }

  /**
   * A package protected method to create a new {@link AutoTrajectory} to be used in an {@link
   * AutoRoutine}.
   *
   * @see AutoRoutine#trajectory(Trajectory)
   */
  @SuppressWarnings("unchecked")
  public <ST extends TrajectorySample<ST>> LoggedAutoTrajectory trajectory(
      Trajectory<ST> trajectory, LoggedAutoRoutine routine, boolean useBindings) {
    // type solidify everything
    final Trajectory<ST> solidTrajectory = trajectory;
    final Consumer<ST> solidController = (Consumer<ST>) this.controller;
    return new LoggedAutoTrajectory(
        trajectory.name(),
        solidTrajectory,
        poseSupplier,
        resetOdometry,
        solidController,
        allianceCtx,
        (TrajectoryLogger<ST>) trajectoryLogger,
        driveSubsystem,
        routine,
        useBindings ? bindings : new AutoBindings());
  }

  /**
   * Creates a new {@link AutoTrajectory} command to be used in an auto routine.
   *
   * <p><b>Important </b>
   *
   * <p>{@link #trajectoryCmd} and {@link #trajectory} methods should not be mixed in the same auto
   * routine. {@link #trajectoryCmd} is used as an escape hatch for teams that don't need the
   * benefits of the {@link #trajectory} method and its {@link Trigger} API. {@link #trajectoryCmd}
   * does not invoke bindings added via calling {@link #bind} or {@link AutoBindings} passed into
   * the factory constructor.
   *
   * @param trajectoryName The name of the trajectory to use.
   * @return A new {@link AutoTrajectory}.
   */
  public Command trajectoryCmd(String trajectoryName) {
    return trajectory(trajectoryName, voidRoutine, false).cmd();
  }

  /**
   * Creates a new {@link AutoTrajectory} command to be used in an auto routine.
   *
   * <p><b>Important </b>
   *
   * <p>{@link #trajectoryCmd} and {@link #trajectory} methods should not be mixed in the same auto
   * routine. {@link #trajectoryCmd} is used as an escape hatch for teams that don't need the
   * benefits of the {@link #trajectory} method and its {@link Trigger} API. {@link #trajectoryCmd}
   * does not invoke bindings added via calling {@link #bind} or {@link AutoBindings} passed into
   * the factory constructor.
   *
   * @param trajectoryName The name of the trajectory to use.
   * @param splitIndex The index of the split trajectory to use.
   * @return A new {@link AutoTrajectory}.
   */
  public Command trajectoryCmd(String trajectoryName, final int splitIndex) {
    return trajectory(trajectoryName, splitIndex, voidRoutine, false).cmd();
  }

  /**
   * Creates a new {@link AutoTrajectory} command to be used in an auto routine.
   *
   * <p><b>Important </b>
   *
   * <p>{@link #trajectoryCmd} and {@link #trajectory} methods should not be mixed in the same auto
   * routine. {@link #trajectoryCmd} is used as an escape hatch for teams that don't need the
   * benefits of the {@link #trajectory} method and its {@link Trigger} API. {@link #trajectoryCmd}
   * does not invoke bindings added via calling {@link #bind} or {@link AutoBindings} passed into
   * the factory constructor.
   *
   * @param <ST> {@link choreo.trajectory.DifferentialSample} or {@link
   *     choreo.trajectory.SwerveSample}
   * @param trajectory The trajectory to use.
   * @return A new {@link AutoTrajectory}.
   */
  public <ST extends TrajectorySample<ST>> Command trajectoryCmd(Trajectory<ST> trajectory) {
    return trajectory(trajectory, voidRoutine, false).cmd();
  }

  /**
   * Creates a command that resets the robot's odometry to the start of a trajectory.
   *
   * @param trajectoryName The name of the trajectory to use.
   * @return A command that resets the robot's odometry.
   */
  public Command resetOdometry(String trajectoryName) {
    return trajectory(trajectoryName, voidRoutine, false).resetOdometry();
  }

  /**
   * Creates a command that resets the robot's odometry to the start of a trajectory.
   *
   * @param trajectoryName The name of the trajectory to use.
   * @param splitIndex The index of the split trajectory to use.
   * @return A command that resets the robot's odometry.
   */
  public Command resetOdometry(String trajectoryName, final int splitIndex) {
    return trajectory(trajectoryName, splitIndex, voidRoutine, false).resetOdometry();
  }

  /**
   * Creates a command that resets the robot's odometry to the start of a trajectory.
   *
   * @param <ST> {@link choreo.trajectory.DifferentialSample} or {@link
   *     choreo.trajectory.SwerveSample}
   * @param trajectory The trajectory to use.
   * @return A command that resets the robot's odometry.
   */
  public <ST extends TrajectorySample<ST>> Command resetOdometry(Trajectory<ST> trajectory) {
    return trajectory(trajectory, voidRoutine, false).resetOdometry();
  }

  /**
   * Binds a command to an event in all trajectories created after this point.
   *
   * @param name The name of the trajectory to bind the command to.
   * @param cmd The command to bind to the trajectory.
   * @return The AutoFactory the method was called from.
   */
  public LoggedAutoFactory bind(String name, Command cmd) {
    bindings.bind(name, cmd);
    return this;
  }

  /**
   * The {@link AutoFactory} caches trajectories with a {@link TrajectoryCache} to avoid reloading
   * the same trajectory multiple times.
   *
   * @return The trajectory cache.
   */
  public LoggedTrajectoryCache cache() {
    return trajectoryCache;
  }

  /**
   * A utility for caching loaded trajectories. This allows for loading trajectories only once, and
   * then reusing them.
   */
  public static class LoggedTrajectoryCache {
    private final Map<String, Trajectory<?>> cache;

    /** Creates a new TrajectoryCache with a normal {@link HashMap} as the cache. */
    public LoggedTrajectoryCache() {
      cache = new HashMap<>();
    }

    /**
     * Creates a new TrajectoryCache with a custom cache.
     *
     * <p>this could be useful if you want to use a concurrent map or a map with a maximum size.
     *
     * @param cache The cache to use.
     */
    public LoggedTrajectoryCache(Map<String, Trajectory<?>> cache) {
      this.cache = cache;
    }

    /**
     * Load a trajectory from the deploy directory. Choreolib expects .traj files to be placed in
     * src/main/deploy/choreo/[trajectoryName].traj.
     *
     * <p>This method will cache the loaded trajectory and reused it if it is requested again.
     *
     * @param trajectoryName the path name in Choreo, which matches the file name in the deploy
     *     directory, file extension is optional.
     * @return the loaded trajectory, or `Optional.empty()` if the trajectory could not be loaded.
     * @see Choreo#loadTrajectory(String)
     */
    public Optional<? extends Trajectory<?>> loadTrajectory(String trajectoryName) {
      if (cache.containsKey(trajectoryName)) {
        return Optional.of(cache.get(trajectoryName));
      } else {
        return Choreo.loadTrajectory(trajectoryName)
            .map(
                trajectory -> {
                  cache.put(trajectoryName, trajectory);
                  return trajectory;
                });
      }
    }

    /**
     * Load a section of a split trajectory from the deploy directory. Choreolib expects .traj files
     * to be placed in src/main/deploy/choreo/[trajectoryName].traj.
     *
     * <p>This method will cache the loaded trajectory and reused it if it is requested again. The
     * trajectory that is split off of will also be cached.
     *
     * @param trajectoryName the path name in Choreo, which matches the file name in the deploy
     *     directory, file extension is optional.
     * @param splitIndex the index of the split trajectory to load
     * @return the loaded trajectory, or `Optional.empty()` if the trajectory could not be loaded.
     * @see Choreo#loadTrajectory(String)
     */
    public Optional<? extends Trajectory<?>> loadTrajectory(String trajectoryName, int splitIndex) {
      // make the key something that could never possibly be a valid trajectory name
      String key = trajectoryName + ".:." + splitIndex;
      if (cache.containsKey(key)) {
        return Optional.of(cache.get(key));
      } else if (cache.containsKey(trajectoryName)) {
        return cache
            .get(trajectoryName)
            .getSplit(splitIndex)
            .map(
                trajectory -> {
                  cache.put(key, trajectory);
                  return trajectory;
                });
      } else {
        return Choreo.loadTrajectory(trajectoryName)
            .flatMap(
                trajectory -> {
                  cache.put(trajectoryName, trajectory);
                  return trajectory
                      .getSplit(splitIndex)
                      .map(
                          split -> {
                            cache.put(key, split);
                            return split;
                          });
                });
      }
    }

    /** Clear the cache. */
    public void clear() {
      cache.clear();
    }
  }
}
