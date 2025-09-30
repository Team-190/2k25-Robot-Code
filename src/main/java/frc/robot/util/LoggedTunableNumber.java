// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard. Automatically executes callbacks when values change.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "TunableNumbers";
  private static final ArrayList<LoggedTunableNumber> strayTunableNumbers = new ArrayList<>();
  private static final List<AutoUpdateGroup> autoUpdateGroups = new ArrayList<>();

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();
  private double lastKnownValue = Double.NaN;
  private List<Runnable> callbacks = new ArrayList<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
    strayTunableNumbers.add(this);
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.TUNING_MODE) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  /**
   * Register a callback to be executed when this tunable number changes
   *
   * @param callback The callback to execute when value changes
   */
  public void onChange(Runnable callback) {
    callbacks.add(callback);
  }

  /** Check if value has changed and execute callbacks if so */
  private void checkAndUpdate() {
    double currentValue = get();
    if (Double.isNaN(lastKnownValue) || currentValue != lastKnownValue) {
      lastKnownValue = currentValue;
      callbacks.forEach(Runnable::run);
    }
  }

  /**
   * Create a group of tunable numbers that will execute a callback when any of them change
   *
   * @param callback The callback to execute when any tunable number in the group changes
   * @param tunableNumbers The tunable numbers to monitor
   * @return AutoUpdateGroup for further configuration if needed
   */
  public static AutoUpdateGroup createGroup(
      Runnable callback, LoggedTunableNumber... tunableNumbers) {
    AutoUpdateGroup group = new AutoUpdateGroup(callback, tunableNumbers);
    autoUpdateGroups.add(group);
    strayTunableNumbers.removeAll(Arrays.asList(tunableNumbers));
    return group;
  }

  /**
   * Create a group of tunable numbers that will execute a callback with values when any of them
   * change
   *
   * @param callback The callback to execute with current values when any tunable number changes
   * @param tunableNumbers The tunable numbers to monitor
   * @return AutoUpdateGroup for further configuration if needed
   */
  public static AutoUpdateGroup createGroup(
      Consumer<double[]> callback, LoggedTunableNumber... tunableNumbers) {
    AutoUpdateGroup group =
        new AutoUpdateGroup(
            () ->
                callback.accept(
                    Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray()),
            tunableNumbers);
    autoUpdateGroups.add(group);
    return group;
  }

  /**
   * Call this method periodically (e.g., in robotPeriodic) to check for changes and execute
   * callbacks
   */
  public static void updateAll() {
    if (Constants.TUNING_MODE) {
      // Update un grouped tunable numbers
      strayTunableNumbers.forEach(LoggedTunableNumber::checkAndUpdate);

      // Update groups
      autoUpdateGroups.forEach(AutoUpdateGroup::checkAndUpdate);
    }
  }

  /** Helper class for managing groups of tunable numbers */
  public static class AutoUpdateGroup {
    private final Runnable callback;
    private final LoggedTunableNumber[] tunableNumbers;
    private final Map<LoggedTunableNumber, Double> lastValues = new HashMap<>();

    private AutoUpdateGroup(Runnable callback, LoggedTunableNumber... tunableNumbers) {
      this.callback = callback;
      this.tunableNumbers = tunableNumbers;
      // Initialize last values
      for (LoggedTunableNumber number : tunableNumbers) {
        lastValues.put(number, number.get());
      }
    }

    private void checkAndUpdate() {
      boolean hasChanged = false;
      for (LoggedTunableNumber number : tunableNumbers) {
        double currentValue = number.get();
        Double lastValue = lastValues.get(number);
        if (lastValue == null || currentValue != lastValue) {
          lastValues.put(number, currentValue);
          hasChanged = true;
        }
      }

      if (hasChanged) {
        callback.run();
      }
    }
  }
}
