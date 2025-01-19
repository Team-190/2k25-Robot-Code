// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.v1_gamma.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

public class Leds extends VirtualSubsystem {
  private static Leds instance;

  // Constants
  private static final int PORT;
  private static final int LENGTH;
  private static final boolean PRIDE_LEDS = false;
  private static final int MIN_LOOP_CYCLE_COUNT = 10;
  private static final int STATIC_SECTION_LENGTH = 3;
  private static final double STROBE_FAST_DURATION = 0.1;
  private static final double STROBE_SLOW_DURATION = 0.2;
  private static final double BREATH_DURATION = 1.0;
  private static final double RAINBOW_CYCLE_LENGTH = 25.0;
  private static final double RAINBOW_DURATION = 0.25;
  private static final double WAVE_EXPONENT = 0.4;
  private static final double WAVE_FAST_CYCLE_LENGTH = 25.0;
  private static final double WAVE_FAST_DURATION = 0.25;
  private static final double WAVE_SLOW_CYCLE_LENGTH = 25.0;
  private static final double WAVE_SLOW_DURATION = 3.0;
  private static final double WAVE_ALLIANCE_CYCLE_LENGTH = 15.0;
  private static final double WAVE_ALLIANCE_DURATION = 2.0;
  private static final double AUTO_FADE_TIME = 2.5; // 3s nominal
  private static final double AUTO_FADE_MAX_TIME = 5.0; // Return to normal

  static {
    switch (Constants.ROBOT) {
      case V1_GAMMA:
        PORT = 0;
        LENGTH = 32;
        break;
      case V1_GAMMA_SIM:
        PORT = 0;
        LENGTH = 0;
        break;
      default:
        PORT = 0;
        LENGTH = 0;
        break;
    }
  }

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean hasNote = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  public boolean lowBatteryAlert = false;
  public static ArrayList<LED_STATE> ledStates = new ArrayList<>();

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kRed;
  private Color secondaryDisabledColor = Color.kWhiteSmoke;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  private Leds() {
    leds = new AddressableLED(PORT);
    buffer = new AddressableLEDBuffer(LENGTH);
    leds.setLength(LENGTH);
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(Color.kWhite, Color.kBlack, System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    lowBatteryAlert = RobotController.getBatteryVoltage() <= 11.8;
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(Color.kWhiteSmoke);
      secondaryDisabledColor = Color.kBlack;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < MIN_LOOP_CYCLE_COUNT) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    setPattern();

    // Update LEDs
    leds.setData(buffer);
  }

  private void setPattern() {
    // Select LED mode
    solid(Color.kBlack); // Default to off
    if (estopped) {
      solid(Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < AUTO_FADE_MAX_TIME) {
        // Auto fade
        solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / AUTO_FADE_TIME), Color.kGreen);
      } else if (lowBatteryAlert) {
        // Low battery
        strobe(Color.kOrangeRed, Color.kBlack, STROBE_FAST_DURATION);
      } else if (PRIDE_LEDS) {
        // Pride stripes
        stripes(
            List.of(
                Color.kBlack,
                Color.kRed,
                Color.kOrangeRed,
                Color.kYellow,
                Color.kGreen,
                Color.kBlue,
                Color.kPurple,
                Color.kBlack,
                new Color(0.15, 0.3, 1.0),
                Color.kDeepPink,
                Color.kWhite,
                Color.kDeepPink,
                new Color(0.15, 0.3, 1.0)),
            3,
            5.0);
        buffer.setLED(STATIC_SECTION_LENGTH, allianceColor);
      } else {
        // Default pattern
        wave(
            allianceColor,
            secondaryDisabledColor,
            WAVE_ALLIANCE_CYCLE_LENGTH,
            WAVE_ALLIANCE_DURATION);
      }

    } else if (DriverStation.isAutonomous()) {
      wave(Color.kGold, Color.kDarkBlue, WAVE_FAST_CYCLE_LENGTH, WAVE_FAST_DURATION);
      if (autoFinished) {
        double fullTime = (double) LENGTH / WAVE_FAST_CYCLE_LENGTH * WAVE_FAST_DURATION;
        solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
      }
    } else { // Enabled
      if (!ledStates.isEmpty()) {
        ledStates.stream()
            .sorted((a, b) -> Integer.compare(a.getPriority(), b.getPriority()))
            .findFirst()
            .ifPresent(state -> state.setPattern(this));
      }
    }
  }

  private void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < LENGTH; i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(LENGTH * percent, 0, LENGTH); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(c1On ? c1 : c2);
  }

  private void breath(Color c1, Color c2, double timestamp) {
    double x = ((timestamp % BREATH_DURATION) / BREATH_DURATION) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < LENGTH; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < LENGTH; i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), WAVE_EXPONENT) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), WAVE_EXPONENT) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(List<Color> colors, int LENGTH, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * LENGTH * colors.size());
    for (int i = 0; i < LENGTH; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / LENGTH) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private void elevatorPattern(DoubleSupplier position, Color c1, Color c2) {
    int litLength = (int) MathUtil.clamp(LENGTH * position.getAsDouble(), 0, LENGTH);
    for (int i = 0; i < LENGTH; i++) {
      if (i < litLength) {
        double wave = Math.sin((i + System.currentTimeMillis() / 100.0) * 0.1);
        Color waveColor =
            new Color((int) (c2.red * wave), (int) (c2.green * wave), (int) (c2.blue * wave));
        buffer.setLED(i, waveColor);
      } else {
        buffer.setLED(i, c1);
      }
    }

    if (litLength > 0) {
      double breath = (Math.sin(System.currentTimeMillis() / 500.0) + 1) / 2;
      Color breathColor =
          new Color((int) (c1.red * breath), (int) (c1.green * breath), (int) (c1.blue * breath));
      buffer.setLED(litLength - 1, breathColor);
    }
  }

  @RequiredArgsConstructor
  public static enum LED_STATE {
    ELEVATOR(
        5,
        (leds) ->
            leds.elevatorPattern(
                () -> 0.0, Color.kBlack, Color.kBlack)), // TODO: replace with supplier for position
    INTAKE(
        1,
        (leds) ->
            leds.stripes(List.of(Color.kAntiqueWhite, Color.kBlack), LENGTH, BREATH_DURATION)),
    SHOOTER(2, (leds) -> leds.strobe(Color.kBlack, Color.kAntiqueWhite, STROBE_SLOW_DURATION)),
    ALIGNED_TO_REEF(
        3,
        (leds) ->
            leds.wave(Color.kGreen, Color.kBlack, WAVE_SLOW_CYCLE_LENGTH, WAVE_SLOW_DURATION)),
    CLIMBING(2, (leds) -> leds.strobe(Color.kYellow, Color.kBlack, STROBE_FAST_DURATION));

    private final int priority;
    private final Consumer<Leds> pattern;

    public int getPriority() {
      return priority;
    }

    public void setPattern(Leds leds) {
      pattern.accept(leds);
    }
  }
}
