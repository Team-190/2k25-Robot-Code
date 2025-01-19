// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shared.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;

public abstract class Leds extends VirtualSubsystem {
  public static class DefaultLeds extends Leds {
    public DefaultLeds() {
      super(0, 0);
    }

    public static Leds getInstance() {
      return new DefaultLeds();
    }
  }

  private static Leds instance;

  // Constants
  private static int PORT;
  private static int LENGTH;
  protected static final boolean PRIDE_LEDS = false;
  protected static final int MIN_LOOP_CYCLE_COUNT = 10;
  protected static final int STATIC_SECTION_LENGTH = 3;
  protected static final double STROBE_FAST_DURATION = 0.1;
  protected static final double STROBE_SLOW_DURATION = 0.2;
  protected static final double BREATH_DURATION = 1.0;
  protected static final double RAINBOW_CYCLE_LENGTH = 25.0;
  protected static final double RAINBOW_DURATION = 0.25;
  protected static final double WAVE_EXPONENT = 0.4;
  protected static final double WAVE_FAST_CYCLE_LENGTH = 25.0;
  protected static final double WAVE_FAST_DURATION = 0.25;
  protected static final double WAVE_SLOW_CYCLE_LENGTH = 25.0;
  protected static final double WAVE_SLOW_DURATION = 3.0;
  protected static final double WAVE_ALLIANCE_CYCLE_LENGTH = 15.0;
  protected static final double WAVE_ALLIANCE_DURATION = 2.0;
  protected static final double AUTO_FADE_TIME = 2.5; // 3s nominal
  protected static final double AUTO_FADE_MAX_TIME = 5.0; // Return to normal

  public static Leds getInstance() {
    if (instance == null) {
      instance = new DefaultLeds();
    }
    return instance;
  }

  // LED IO
  protected final AddressableLED leds;
  protected final AddressableLEDBuffer buffer;
  protected final Notifier loadingNotifier;

  protected Leds(int LENGTH, int PORT) {
    this.LENGTH = LENGTH;
    this.PORT = PORT;
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

  public synchronized void periodic() {}

  public void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < LENGTH; i++) {
        buffer.setLED(i, color);
      }
    }
  }

  public void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(LENGTH * percent, 0, LENGTH); i++) {
      buffer.setLED(i, color);
    }
  }

  public void strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(c1On ? c1 : c2);
  }

  public void breath(Color c1, Color c2, double timestamp) {
    double x = ((timestamp % BREATH_DURATION) / BREATH_DURATION) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

  public void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < LENGTH; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  public void wave(Color c1, Color c2, double cycleLength, double duration) {
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

  public void stripes(List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = 0; i < length; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  public void elevatorPattern(DoubleSupplier position, Color c1, Color c2) {
    int litLength =
        (int) MathUtil.clamp(Units.metersToInches(position.getAsDouble()) / 0.5, 0, LENGTH);
    wave(c1, c2, litLength, 0.1);
    if (litLength > 0) {
      breath(c1, c2, System.currentTimeMillis() / 1000.0);
    }
  }
}
