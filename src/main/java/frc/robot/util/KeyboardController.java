package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

public class KeyboardController {
  private final int port;
  private final int numButtons;

  private NetworkTableInstance inst;
  private NetworkTable keyboardTable;
  BooleanSubscriber[] buttonSubscribers;
  BooleanSubscriber isConnectedSubscriber;

  private final Map<EventLoop, Map<Integer, Trigger>> m_buttonCache = new HashMap<>();

  public KeyboardController(int port) {
    this(port, 80);
  }

  public KeyboardController(int port, int numButtons) {
    this.port = port;
    this.numButtons = numButtons;
    this.inst = NetworkTableInstance.getDefault();
    this.keyboardTable = inst.getTable("AdvantageKit/Keyboard/" + port);

    buttonSubscribers = new BooleanSubscriber[numButtons];
    for (int i = 0; i < numButtons; i++) {
      buttonSubscribers[i] = keyboardTable.getBooleanTopic(i + "").subscribe(false);
    }
    isConnectedSubscriber = keyboardTable.getBooleanTopic("isConnected").subscribe(false);
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #button(int, EventLoop)
   */
  public Trigger button(int button) {
    return button(button, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the button's digital signal attached to the given loop.
   */
  public Trigger button(int button, EventLoop loop) {
    var cache = m_buttonCache.computeIfAbsent(loop, k -> new HashMap<>());
    return cache.computeIfAbsent(button, k -> new Trigger(loop, () -> getRawButton(k)));
  }

  /**
   * Get if the HID is connected.
   *
   * @return true if the HID is connected
   */
  public boolean isConnected() {
    return true;
  }

  private boolean getRawButton(int button) {
    return buttonSubscribers[button].get();
  }
}
