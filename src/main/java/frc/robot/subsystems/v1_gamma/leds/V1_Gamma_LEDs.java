package frc.robot.subsystems.v1_gamma.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.shared.leds.Leds;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;

public class V1_Gamma_LEDs extends Leds {
  private static Leds instance;

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean hasNote = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  public boolean lowBatteryAlert = false;

  @AutoLogOutput(key = "LEDs/Led States")
  public static ArrayList<LED_STATE> ledStates = new ArrayList<>();

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kRed;
  private Color secondaryDisabledColor = Color.kWhiteSmoke;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  private static final int LENGTH = 60;
  private static final int PORT = 0;

  public V1_Gamma_LEDs() {
    super(LENGTH, PORT);
  }

  public static Leds getInstance() {
    if (instance == null) {
      instance = new V1_Gamma_LEDs();
    }
    return instance;
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
    if (loopCycleCount < Leds.MIN_LOOP_CYCLE_COUNT) {
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
