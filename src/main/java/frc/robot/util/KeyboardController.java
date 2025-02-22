package frc.robot.util;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

public class KeyboardController {
  private final int numButtons;

  private NetworkTableInstance inst;
  private NetworkTable keyboardTable;
  BooleanSubscriber[] buttonSubscribers;
  BooleanSubscriber isConnectedSubscriber;

  private final Map<EventLoop, Map<Integer, Trigger>> m_buttonCache = new HashMap<>();

  // Group of triggers for each subsystem
  private final Funnel funnel;
  private final EndEffector endEffector;
  private final Elevator elevator;
  private final Climber climber;
  private final Scoring scoring;

  public KeyboardController(int port) {
    this(port, 80);
  }

  public KeyboardController(int port, int numButtons) {
    this.numButtons = numButtons;
    this.inst = NetworkTableInstance.getDefault();
    this.keyboardTable = inst.getTable("/AdvantageKit/DriverStation/Keyboard" + port);

    this.funnel = new Funnel(this);
    this.endEffector = new EndEffector(this);
    this.elevator = new Elevator(this);
    this.climber = new Climber(this);
    this.scoring = new Scoring(this);

    buttonSubscribers = new BooleanSubscriber[this.numButtons];
    for (int i = 0; i < this.numButtons; i++) {
      buttonSubscribers[i] = keyboardTable.getBooleanTopic(String.valueOf(i)).subscribe(false);
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
    return cache.computeIfAbsent(
        button, k -> new Trigger(loop, () -> getRawButton(k) && isConnected()));
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param row the row index
   * @param col the column index
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #button(int, EventLoop)
   */
  public Trigger button(int row, int col) {
    return button(8 * (col - 1) + row - 1);
  }

  /**
   * Get if the HID is connected.
   *
   * @return true if the HID is connected
   */
  public boolean isConnected() {
    return isConnectedSubscriber.get();
  }

  private boolean getRawButton(int button) {
    return buttonSubscribers[button].get();
  }

  public Funnel funnel() {
    return funnel;
  }

  public EndEffector endEffector() {
    return endEffector;
  }

  public Elevator elevator() {
    return elevator;
  }

  public Climber climber() {
    return climber;
  }

  public Scoring scoring() {
    return scoring;
  }

  public Trigger resetHeading() {
    return button(2, 10);
  }

  public static final record Funnel(KeyboardController controller) {
    public Trigger wingsClose() {
      return controller.button(2, 1);
    }

    public Trigger wingsIntake() {
      return controller.button(3, 2);
    }

    public Trigger rollerWheelsIn() {
      return controller.button(4, 1);
    }

    public Trigger incrementClosedSetpoint() {
      return controller.button(2, 2);
    }

    public Trigger incrementIntakeSetpoint() {
      return controller.button(3, 2);
    }

    public Trigger rollerWheelsOut() {
      return controller.button(4, 2);
    }

    public Trigger decrementClosedSetpoint() {
      return controller.button(2, 3);
    }

    public Trigger decrementIntakeSetpoint() {
      return controller.button(3, 3);
    }

    public Trigger funnelSensorToggle() {
      return controller.button(2, 4);
    }

    public Trigger incrementRollerWheelsSpeed() {
      return controller.button(4, 3);
    }

    public Trigger decrementRollerWheelsSpeed() {
      return controller.button(4, 4);
    }
  }

  public static final record EndEffector(KeyboardController controller) {
    public Trigger wheelsIn() {
      return controller.button(6, 1);
    }

    public Trigger wheelsOut() {
      return controller.button(6, 2);
    }

    public Trigger toggleSensor() {
      return controller.button(7, 4);
    }

    public Trigger incrementSpeed() {
      return controller.button(6, 3);
    }

    public Trigger decrementSpeed() {
      return controller.button(6, 4);
    }

    public Trigger eject() {
      return controller.button(7, 1);
    }
  }

  public static final record Elevator(KeyboardController controller) {
    public Trigger stow() {
      return controller.button(2, 5);
    }

    public Trigger raise() {
      return controller.button(3, 5);
    }

    public Trigger primeL4() {
      return controller.button(4, 5);
    }

    public Trigger primeL3() {
      return controller.button(5, 5);
    }

    public Trigger primeL2() {
      return controller.button(6, 5);
    }

    public Trigger primeL1() {
      return controller.button(7, 5);
    }

    public Trigger increaseStowSetpoint() {
      return controller.button(2, 6);
    }

    public Trigger decreaseStowSetpoint() {
      return controller.button(2, 7);
    }

    public Trigger increaseL4Setpoint() {
      return controller.button(4, 6);
    }

    public Trigger decreaseL4Setpoint() {
      return controller.button(4, 7);
    }

    public Trigger increaseL3Setpoint() {
      return controller.button(5, 6);
    }

    public Trigger decreaseL3Setpoint() {
      return controller.button(5, 7);
    }

    public Trigger increaseL2Setpoint() {
      return controller.button(6, 6);
    }

    public Trigger decreaseL2Setpoint() {
      return controller.button(6, 7);
    }

    public Trigger increaseL1Setpoint() {
      return controller.button(7, 6);
    }

    public Trigger decreaseL1Setpoint() {
      return controller.button(7, 7);
    }
  }

  public static final record Climber(KeyboardController controller) {
    public Trigger deployLower() {
      return controller.button(6, 8);
    }

    public Trigger stingerOut() {
      return controller.button(7, 8);
    }

    public Trigger incrementWintchOut() {
      return controller.button(6, 9);
    }

    public Trigger incrementWintchIn() {
      return controller.button(6, 10);
    }
  }

  public static final record Scoring(KeyboardController controller) {
    public Trigger primeLeft() {
      return controller.button(4, 8);
    }

    public Trigger primeRight() {
      return controller.button(4, 9);
    }

    public Trigger track() {
      return controller.button(4, 10);
    }
  }
}
