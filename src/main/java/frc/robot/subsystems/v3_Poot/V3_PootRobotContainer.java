package frc.robot.subsystems.v3_Poot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.CompositeCommands.SharedCommands;
import frc.robot.commands.CompositeCommands.V3_PootCompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.*;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.elevator.ElevatorIO;
import frc.robot.subsystems.shared.elevator.ElevatorIOSim;
import frc.robot.subsystems.shared.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.shared.vision.VisionConstants.RobotCameras;
import frc.robot.subsystems.v3_Poot.climber.V3_PootClimber;
import frc.robot.subsystems.v3_Poot.climber.V3_PootClimberIO;
import frc.robot.subsystems.v3_Poot.climber.V3_PootClimberIOSim;
import frc.robot.subsystems.v3_Poot.climber.V3_PootClimberIOTalonFX;
import frc.robot.subsystems.v3_Poot.leds.V3_PootLEDs;
import frc.robot.subsystems.v3_Poot.superstructure.V3_PootSuperstructure;
import frc.robot.subsystems.v3_Poot.superstructure.V3_PootSuperstructureStates;
import frc.robot.subsystems.v3_Poot.superstructure.intake.V3_PootIntake;
import frc.robot.subsystems.v3_Poot.superstructure.intake.V3_PootIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Poot.superstructure.intake.V3_PootIntakeIO;
import frc.robot.subsystems.v3_Poot.superstructure.intake.V3_PootIntakeIOSim;
import frc.robot.subsystems.v3_Poot.superstructure.manipulator.V3_PootManipulator;
import frc.robot.subsystems.v3_Poot.superstructure.manipulator.V3_PootManipulatorConstants.ManipulatorRollerState;
import frc.robot.subsystems.v3_Poot.superstructure.manipulator.V3_PootManipulatorIO;
import frc.robot.subsystems.v3_Poot.superstructure.manipulator.V3_PootManipulatorIOSim;
import frc.robot.subsystems.v3_Poot.superstructure.manipulator.V3_PootManipulatorIOTalonFX;
import frc.robot.util.LTNUpdater;
import frc.robot.util.LoggedChoreo.ChoreoChooser;
import frc.robot.util.LoggedTunableNumber;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class V3_PootRobotContainer implements RobotContainer {
  // Subsystems
  private Drive drive;
  private ElevatorFSM elevator;
  private V3_PootIntake intake;
  private V3_PootManipulator manipulator;
  private V3_PootSuperstructure superstructure;
  private V3_PootClimber climber;
  private V3_PootLEDs leds;
  private Vision vision;

  // Controller
  private static final CommandXboxController driver = new CommandXboxController(0);
  private static final CommandXboxController operator = new CommandXboxController(1);

  // Auto chooser
  private final ChoreoChooser autoChooser = new ChoreoChooser();

  public V3_PootRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V3_POOT:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0, DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(1, DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(2, DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(3, DriveConstants.BACK_RIGHT));
          elevator = new Elevator(new ElevatorIOTalonFX()).getFSM();
          //   intake = new V3_PootIntake(new V3_PootIntakeIOTalonFX());
          manipulator = new V3_PootManipulator(new V3_PootManipulatorIOTalonFX());
          climber = new V3_PootClimber(new V3_PootClimberIOTalonFX());
          //   superstructure = new V3_PootSuperstructure(elevator, intake, manipulator);
          leds = new V3_PootLEDs();
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                  RobotCameras.V3_LL_TEST);
          break;
        case V3_POOT_SIM:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          elevator = new Elevator(new ElevatorIOSim()).getFSM();
          intake = new V3_PootIntake(new V3_PootIntakeIOSim());
          manipulator = new V3_PootManipulator(new V3_PootManipulatorIOSim());
          climber = new V3_PootClimber(new V3_PootClimberIOSim());
          superstructure = new V3_PootSuperstructure(elevator, intake, manipulator);
          leds = new V3_PootLEDs();
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                  RobotCameras.V3_POOT_CAMS);
          break;
        default:
          break;
      }
    }

    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {}).getFSM();
    }
    if (intake == null) {
      intake = new V3_PootIntake(new V3_PootIntakeIO() {});
    }
    if (manipulator == null) {
      manipulator = new V3_PootManipulator(new V3_PootManipulatorIO() {});
    }
    if (climber == null) {
      climber = new V3_PootClimber(new V3_PootClimberIO() {});
    }
    if (leds == null) {
      leds = new V3_PootLEDs();
    }
    if (superstructure == null) {
      superstructure = new V3_PootSuperstructure(elevator, intake, manipulator);
    }

    LTNUpdater.registerAll(drive, elevator, intake, manipulator);

    configureButtonBindings();
    configureAutos();
  }

  /**
   * Configure the button bindings for the robot. This method is called in the constructor and is
   * responsible for setting up the default commands for each button on the controllers.
   */
  private void configureButtonBindings() {
    Trigger elevatorStow =
        new Trigger(
            () ->
                elevator.getPosition().equals(ElevatorPositions.CORAL_INTAKE)
                    || elevator.getPosition().equals(ElevatorPositions.STOW));
    Trigger elevatorNotStow =
        new Trigger(
            () ->
                !elevator.getPosition().equals(ElevatorPositions.CORAL_INTAKE)
                    && !elevator.getPosition().equals(ElevatorPositions.STOW));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false,
            () -> false,
            driver.povRight()));

    driver.povDown().onTrue(SharedCommands.resetHeading(drive));

    driver.y().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L4));
    driver.x().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L3));
    driver.b().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L2));
    driver.a().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L1));

    driver
        .y()
        .and(elevatorNotStow)
        .onTrue(V3_PootCompositeCommands.setDynamicReefHeight(ReefState.L4, superstructure));
    driver
        .x()
        .and(elevatorNotStow)
        .onTrue(V3_PootCompositeCommands.setDynamicReefHeight(ReefState.L3, superstructure));
    driver
        .b()
        .and(elevatorNotStow)
        .onTrue(V3_PootCompositeCommands.setDynamicReefHeight(ReefState.L2, superstructure));
    driver
        .a()
        .and(elevatorNotStow)
        .onTrue(V3_PootCompositeCommands.setDynamicReefHeight(ReefState.L1, superstructure));

    driver
        .rightTrigger(0.5)
        .whileTrue(V3_PootCompositeCommands.optimalAutoScoreCoralSequence(drive, superstructure));
    driver
        .leftTrigger(0.5)
        .whileTrue(
            V3_PootCompositeCommands.intakeCoralDriverSequence(superstructure, intake, manipulator))
        .onFalse(
            V3_PootCompositeCommands.postIntakeCoralSequence(superstructure, intake, manipulator));

    driver
        .leftBumper()
        .whileTrue(V3_PootCompositeCommands.intakeAlgaeFloor(superstructure, manipulator))
        .onFalse(
            Commands.either(
                superstructure.runGoal(V3_PootSuperstructureStates.STOW_UP),
                superstructure.runGoal(V3_PootSuperstructureStates.STOW_DOWN),
                () -> RobotState.isHasAlgae()));
    driver.rightBumper().onTrue(Commands.runOnce(() -> RobotState.toggleReefPost()));

    driver.povUp().onTrue(superstructure.setPosition());
    driver.povDown().onTrue(SharedCommands.resetHeading(drive));
    // driver.povLeft().onTrue(DriveCommands.inchMovement(drive, -0.5, .07));

    driver
        .leftStick()
        .onTrue(V3_PootCompositeCommands.optimalAutoScoreCoralSequence(drive, superstructure));

    driver
        .back()
        .whileTrue(V3_PootCompositeCommands.intakeAlgaeFromReef(drive, superstructure))
        .whileFalse(V3_PootCompositeCommands.postIntakeAlgaeFromReef(drive, superstructure));

    driver
        .start()
        .whileTrue(
            V3_PootCompositeCommands.dropAlgae(
                drive,
                elevator,
                manipulator,
                intake,
                superstructure,
                () -> RobotState.getReefAlignData().algaeIntakeHeight()));

    operator.y().onTrue(SharedCommands.setStaticReefHeight(ReefState.L4));
    operator.x().onTrue(SharedCommands.setStaticReefHeight(ReefState.L3));
    operator.b().onTrue(SharedCommands.setStaticReefHeight(ReefState.L2));
    operator.a().onTrue(SharedCommands.setStaticReefHeight(ReefState.L1));

    operator
        .y()
        .and(elevatorNotStow)
        .onTrue(V3_PootCompositeCommands.setDynamicReefHeight(ReefState.L4, superstructure));
    operator
        .x()
        .and(elevatorNotStow)
        .onTrue(V3_PootCompositeCommands.setDynamicReefHeight(ReefState.L3, superstructure));
    operator
        .b()
        .and(elevatorNotStow)
        .onTrue(V3_PootCompositeCommands.setDynamicReefHeight(ReefState.L2, superstructure));
    operator
        .a()
        .and(elevatorNotStow)
        .onTrue(V3_PootCompositeCommands.setDynamicReefHeight(ReefState.L1, superstructure));

    operator
        .rightTrigger(0.5)
        .whileTrue(
            superstructure.override(
                () -> manipulator.setRollerGoal(ManipulatorRollerState.SCORE_CORAL), 0.4));

    operator.leftBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)));
    operator.rightBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)));

    operator
        .povUp()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> manipulator.setArmGoal(Rotation2d.fromDegrees(90))),
                Commands.runOnce(() -> elevator.setPosition(() -> ReefState.HANDOFF)),
                Commands.runOnce(() -> intake.setPivotGoal(IntakePivotState.L1)),
                climber.releaseClimber()));
    operator
        .povDown()
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> RobotState.setClimberReady(false)),
                Commands.runOnce(() -> elevator.setPosition(() -> ReefState.STOW)),
                climber.winchClimber()));
    operator
        .povLeft()
        .whileTrue(superstructure.runGoal(V3_PootSuperstructureStates.PROCESSOR))
        .onFalse(
            superstructure
                .runActionWithTimeout(V3_PootSuperstructureStates.PROCESSOR_SCORE, 1)
                .finallyDo(() -> RobotState.setHasAlgae(false)));

    operator.start().whileTrue(superstructure.runGoal(V3_PootSuperstructureStates.BARGE));

    operator.back().onTrue(V3_PootCompositeCommands.optimalScoreBarge(superstructure));

    Trigger trigger = driver.povLeft();
    Trigger trigger2 = driver.povRight();

    trigger.onTrue(superstructure.everythingsFucked(trigger2));
  }

  private void configureAutos() {
    autoChooser.addCmd(
        "Drive FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Wheel Radius Characterization", () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addRoutine(
        "4 Piece Right Early Madtown",
        () -> AutonomousCommands.autoERight(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "4 Piece Right Late Madtown",
        () -> AutonomousCommands.autoERightBack(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "4 Piece Left Early Madtown",
        () -> AutonomousCommands.autoELeft(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "4 Piece Left Late Madtown",
        () -> AutonomousCommands.autoELeftBack(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "Algae", () -> AutonomousCommands.autoFLeft(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "1 piece do nothing",
        () -> AutonomousCommands.autoFLeftMinimal(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "Range Benchmark 1", () -> AutonomousCommands.rangeBenchOne(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "Range Benchmark 2", () -> AutonomousCommands.rangeBenchTwo(drive, superstructure, intake, manipulator));    
    autoChooser.addRoutine(
        "Range Benchmark 3", () -> AutonomousCommands.rangeBenchThree(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "Range Benchmark 4", () -> AutonomousCommands.rangeBenchFour(drive, superstructure, intake, manipulator));
    autoChooser.addRoutine(
        "Range Benchmark 5", () -> AutonomousCommands.rangeBenchFive(drive, superstructure, intake, manipulator));
    SmartDashboard.putData("Autonomous Modes", autoChooser);
  }

  /**
   * Periodic function for the robot. This function is called every 20ms, and is responsible for
   * updating the robot's state and logging relevant data.
   */
  @Override
  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getModulePositions(),
        vision.getCameras());

    LoggedTunableNumber.updateAll();

    Logger.recordOutput(
        "Component Poses",
        V3_PootMechanism3d.getPoses(
            elevator.getPositionMeters(), intake.getPivotAngle(), manipulator.getArmAngle()));

    if (!Constants.getMode().equals(Constants.Mode.REAL)) {
      Logger.recordOutput(
          "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
      Logger.recordOutput(
          "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    }
  }

  /**
   * Returns the autonomous command for the robot. This command will be scheduled for the entire
   * autonomous period.
   *
   * @return the autonomous command for the robot
   */
  @Override
  public Command getAutonomousCommand() {
    return AutonomousCommands.speedBenchThree(drive, superstructure, intake, manipulator).cmd();
  }
}
