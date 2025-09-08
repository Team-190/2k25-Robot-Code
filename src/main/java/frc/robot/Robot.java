package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotState.RobotMode;
import frc.robot.subsystems.v0_Funky.V0_FunkyRobotContainer;
import frc.robot.subsystems.v0_GompeivisionTest.v0_GompeivisionTestRobotContainer;
import frc.robot.subsystems.v0_Whiplash.V0_WhiplashRobotContainer;
import frc.robot.subsystems.v1_StackUp.V1_StackUpRobotContainer;
import frc.robot.subsystems.v2_Redundancy.V2_RedundancyRobotContainer;
import frc.robot.subsystems.v3_Epsilon.V3_EpsilonRobotContainer;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.CanivoreReader;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.VirtualSubsystem;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double lowBatteryVoltage = 10.0;
  private static final double lowBatteryDisabledTime = 1.5;

  @SuppressWarnings("unused")
  private static final double canErrorTimeThreshold = 0.5;

  @SuppressWarnings("unused")
  private static final double canivoreErrorTimeThreshold = 0.5;

  private static double startupTimestamp = Double.NEGATIVE_INFINITY;

  private final Timer canErrorTimer = new Timer();
  private final Timer canErrorTimerInitial = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private final Timer disabledTimer = new Timer();
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.WARNING);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);

  @SuppressWarnings("unused")
  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);

  @SuppressWarnings("unused")
  private final Alert canivoreErrorAlert =
      new Alert("CANivore errors detected, robot may not be controllable.", AlertType.ERROR);

  @SuppressWarnings("unused")
  private final CanivoreReader canivoreReader = new CanivoreReader("Drive");

  private static final double loopOverrunWarningTimeout = 1;

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    super(Constants.LOOP_PERIOD_SECONDS);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SignalLogger.enableAutoLogging(false);
    LiveWindow.disableAllTelemetry();
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        // setUseTiming(false);
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Start timers
    canErrorTimer.restart();
    canErrorTimerInitial.restart();
    canivoreErrorTimer.restart();
    disabledTimer.reset();
    disabledTimer.start();

    RobotController.setBrownoutVoltage(6);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer =
        switch (Constants.ROBOT) {
          case V0_WHIPLASH, V0_WHIPLASH_SIM -> new V0_WhiplashRobotContainer();
          case V0_FUNKY, V0_FUNKY_SIM -> new V0_FunkyRobotContainer();
          case V0_GOMPEIVISION_TEST,
              V0_GOMPEIVISION_TEST_SIM -> new v0_GompeivisionTestRobotContainer();
          case V1_STACKUP, V1_STACKUP_SIM -> new V1_StackUpRobotContainer();
          case V2_REDUNDANCY, V2_REDUNDANCY_SIM -> new V2_RedundancyRobotContainer();
          case V3_EPSILON, V3_EPSILON_SIM -> new V3_EpsilonRobotContainer();
          default -> new RobotContainer() {};
        };

    Shuffleboard.selectTab("Autonomous");
    DriverStation.silenceJoystickConnectionWarning(true);

    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

    // Silence Rotation2d warnings
    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    startupTimestamp = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.

    InternalLoggedTracer.reset();
    PhoenixUtil.refreshAll();
    InternalLoggedTracer.record("Refresh Status Signals", "Robot");

    InternalLoggedTracer.reset();
    robotContainer.robotPeriodic();
    InternalLoggedTracer.record("Robot Container Periodic", "Robot");

    InternalLoggedTracer.reset();
    VirtualSubsystem.periodicAll();
    InternalLoggedTracer.record("Virtual Subsystem Periodic", "Robot");

    InternalLoggedTracer.reset();
    CommandScheduler.getInstance().run();
    InternalLoggedTracer.record("Command Scheduler Run", "Robot");

    // Check logging fault
    InternalLoggedTracer.reset();
    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());
    InternalLoggedTracer.record("Check Logging Fault", "Robot");

    // Update low battery alert
    InternalLoggedTracer.reset();
    if (RobotState.RobotMode.enabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() < lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }
    InternalLoggedTracer.record("Check Battery Alert", "Robot");

    // Check CAN status
    // LoggedTracer.reset();
    // var canStatus = RobotController.getCANStatus();
    // if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
    //   canErrorTimer.restart();
    // }
    // canErrorAlert.set(
    //     !canErrorTimer.hasElapsed(canErrorTimeThreshold)
    //         && !canErrorTimerInitial.hasElapsed(canErrorTimeThreshold));

    // // Log CANivore status
    // if (Constants.getMode() == Constants.Mode.REAL) {
    //   var canivoreStatus = canivoreReader.getStatus();
    //   if (canivoreStatus.isPresent()) {
    //     Logger.recordOutput(
    //         NTPrefixes.CANIVORE_STATUS + "Status", canivoreStatus.get().Status.getName());
    //     Logger.recordOutput(
    //         NTPrefixes.CANIVORE_STATUS + "Utilization", canivoreStatus.get().BusUtilization);
    //     Logger.recordOutput(
    //         NTPrefixes.CANIVORE_STATUS + "OffCount", canivoreStatus.get().BusOffCount);
    //     Logger.recordOutput(
    //         NTPrefixes.CANIVORE_STATUS + "TxFullCount", canivoreStatus.get().TxFullCount);
    //     Logger.recordOutput(
    //         NTPrefixes.CANIVORE_STATUS + "ReceiveErrorCount", canivoreStatus.get().REC);
    //     Logger.recordOutput(
    //         NTPrefixes.CANIVORE_STATUS + "TransmitErrorCount", canivoreStatus.get().TEC);
    //     if (!canivoreStatus.get().Status.isOK()
    //         || canStatus.transmitErrorCount > 0
    //         || canStatus.receiveErrorCount > 0) {
    //       canivoreErrorTimer.restart();
    //     }
    //   }
    //   canivoreErrorAlert.set(
    //       !canivoreErrorTimer.hasElapsed(canivoreErrorTimeThreshold)
    //           && !canErrorTimerInitial.hasElapsed(canErrorTimeThreshold));
    // }
    // LoggedTracer.record("Check CANivore Status", "Robot");
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    RobotState.setMode(RobotMode.DISABLED);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    InternalLoggedTracer.reset();
    RobotState.setMode(RobotState.RobotMode.AUTO);
    InternalLoggedTracer.record("Set Robotstate Mode", "Robot");

    InternalLoggedTracer.reset();
    RobotState.setReefHeight(ReefState.L4);
    InternalLoggedTracer.record("Set Reef Height", "Robot");

    InternalLoggedTracer.reset();
    autonomousCommand = robotContainer.getAutonomousCommand();
    InternalLoggedTracer.record("Set Autonomous Command", "Robot");

    InternalLoggedTracer.reset();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    InternalLoggedTracer.record("Schedule Autonomous Command", "Robot");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    RobotState.setMode(RobotMode.TELEOP);
    Shuffleboard.selectTab("Teleoperated");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public static boolean isJitting() {
    return Timer.getFPGATimestamp() - startupTimestamp <= 45.0;
  }
}
