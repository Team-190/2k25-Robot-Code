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
    return button(73);
  }

  public static final record Funnel(KeyboardController controller) {
    public Trigger wingsClose() {
      return controller.button(1);
    }

    public Trigger wingsIntake() {
      return controller.button(2);
    }

    public Trigger wingsStingerOut() {
      return controller.button(3);
    }

    public Trigger wheelsIn() {
      return controller.button(4);
    }

    public Trigger incrementClosedSetpoint() {
      return controller.button(9);
    }

    public Trigger incrementIntakeSetpoint() {
      return controller.button(10);
    }

    public Trigger incrementStingerOutSetpoint() {
      return controller.button(11);
    }

    public Trigger wheelsOut() {
      return controller.button(12);
    }

    public Trigger decrementClosedSetpoint() {
      return controller.button(17);
    }

    public Trigger decrementIntakeSetpoint() {
      return controller.button(18);
    }

    public Trigger decrementStingerOutSetpoint() {
      return controller.button(19);
    }

    public Trigger incrementFunnelWheelsSpeed() {
      return controller.button(20);
    }

    public Trigger funnelSensorToggle() {
      return controller.button(25);
    }

    public Trigger decrementFunnelWheelsSpeed() {
      return controller.button(28);
    }
  }

  public static final record EndEffector(KeyboardController controller) {
    public Trigger wheelsIn() {
      return controller.button(6);
    }

    public Trigger wheelsOut() {
      return controller.button(14);
    }

    public Trigger toggleSensor() {
      return controller.button(7);
    }

    public Trigger incrementSpeed() {
      return controller.button(22);
    }

    public Trigger decrementSpeed() {
      return controller.button(30);
    }
  }

  public static final record Elevator(KeyboardController controller) {
    public Trigger stow() {
      return controller.button(33);
    }

    public Trigger raise() {
      return controller.button(34);
    }

    public Trigger primeL4() {
      return controller.button(35);
    }

    public Trigger primeL3() {
      return controller.button(36);
    }

    public Trigger primeL2() {
      return controller.button(37);
    }

    public Trigger primeL1() {
      return controller.button(38);
    }

    public Trigger increaseStowSetpoint() {
      return controller.button(41);
    }

    public Trigger decreaseStowSetpoint() {
      return controller.button(49);
    }

    public Trigger increaseL4Setpoint() {
      return controller.button(43);
    }

    public Trigger decreaseL4Setpoint() {
      return controller.button(51);
    }

    public Trigger increaseL3Setpoint() {
      return controller.button(44);
    }

    public Trigger decreaseL3Setpoint() {
      return controller.button(52);
    }

    public Trigger increaseL2Setpoint() {
      return controller.button(45);
    }

    public Trigger decreaseL2Setpoint() {
      return controller.button(53);
    }

    public Trigger increaseL1Setpoint() {
      return controller.button(46);
    }

    public Trigger decreaseL1Setpoint() {
      return controller.button(54);
    }
  }

  public static final record Climber(KeyboardController controller) {
    public Trigger deployLower() {
      return controller.button(61);
    }

    public Trigger incrementWintchOut() {
      return controller.button(69);
    }

    public Trigger incrementWintchIn() {
      return controller.button(77);
    }
  }

  public static final record Scoring(KeyboardController controller) {
    public Trigger primeLeft() {
      return controller.button(59);
    }

    public Trigger primeRight() {
      return controller.button(67);
    }

    public Trigger track() {
      return controller.button(75);
    }
  }
}
