package frc.robot.subsystems.v1_gamma.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.RobotState;
import frc.robot.subsystems.shared.leds.Leds;
import lombok.Setter;

public class V1_Gamma_LEDs extends Leds {
  private static Leds instance;

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean autoFinished = false;
  public boolean lowBatteryAlert = false;

  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  private static final int LENGTH = 68;
  private static final int PORT = 0;
  private static final int RIGHT_LENGTH_START = 0;
  private static final int RIGHT_LENGTH_END = 33;
  private static final int LEFT_LENGTH_START = 34;
  private static final int LEFT_LENGTH_END = 68;

  @Setter private static boolean isIntaking = false;
  @Setter private static boolean isAutoAligning = false;

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
    lowBatteryAlert = RobotController.getBatteryVoltage() <= 12.4;
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
      } else {
        rainbow(LENGTH, 5.0);
      }
    } else if (DriverStation.isEnabled()) {
      if (isAutoAligning) {
        rainbow(LENGTH, 0.5);
      } else {
        if (RobotState.getOperatorInputData().currentReefPost().equals(ReefPost.RIGHT)) {
          if (isIntaking) {
            solid(Color.kAqua, LEFT_LENGTH_START, LEFT_LENGTH_END);
          } else {
            breath(
                Color.kBlack,
                Color.kDarkViolet,
                Timer.getFPGATimestamp(),
                LEFT_LENGTH_START,
                LEFT_LENGTH_END);
          }
        } else if (RobotState.getOperatorInputData().currentReefPost().equals(ReefPost.LEFT)) {
          if (isIntaking) {
            solid(Color.kAqua, RIGHT_LENGTH_START, RIGHT_LENGTH_END);
          } else {
            breath(
                Color.kBlack,
                Color.kDarkViolet,
                Timer.getFPGATimestamp(),
                RIGHT_LENGTH_START,
                RIGHT_LENGTH_END);
          }
        }
      }
    }
  }
}
