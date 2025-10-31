package frc.robot.subsystems.v3_Epsilon;

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
import frc.robot.commands.CompositeCommands.V3_EpsilonCompositeCommands;
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
import frc.robot.subsystems.v3_Epsilon.climber.V3_EpsilonClimber;
import frc.robot.subsystems.v3_Epsilon.climber.V3_EpsilonClimberIO;
import frc.robot.subsystems.v3_Epsilon.climber.V3_EpsilonClimberIOSim;
import frc.robot.subsystems.v3_Epsilon.climber.V3_EpsilonClimberIOTalonFX;
import frc.robot.subsystems.v3_Epsilon.leds.V3_EpsilonLEDs;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructure;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureStates;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants.IntakePivotState;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIO;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIOSim;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIOTalonFX;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorRollerState;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorIO;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorIOSim;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorIOTalonFX;
import frc.robot.util.LTNUpdater;
import frc.robot.util.LoggedChoreo.ChoreoChooser;
import frc.robot.util.LoggedTunableNumber;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonRobotContainer implements RobotContainer {
  // Subsystems
  private Drive drive;
  private ElevatorFSM elevator;
  private V3_EpsilonIntake intake;
  private V3_EpsilonManipulator manipulator;
  private V3_EpsilonSuperstructure superstructure;
  private V3_EpsilonClimber climber;
  private V3_EpsilonLEDs leds;
  private Vision vision;

  // Controller
  private static final CommandXboxController driver = new CommandXboxController(0);
  private static final CommandXboxController operator = new CommandXboxController(1);

  // Auto chooser
  private final ChoreoChooser autoChooser = new ChoreoChooser();

  public V3_EpsilonRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V3_EPSILON:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0, DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(1, DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(2, DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(3, DriveConstants.BACK_RIGHT));
          elevator = new Elevator(new ElevatorIOTalonFX()).getFSM();
          intake = new V3_EpsilonIntake(new V3_EpsilonIntakeIOTalonFX());
          manipulator = new V3_EpsilonManipulator(new V3_EpsilonManipulatorIOTalonFX());
          climber = new V3_EpsilonClimber(new V3_EpsilonClimberIOTalonFX());
          superstructure = new V3_EpsilonSuperstructure(elevator, intake, manipulator);
          leds = new V3_EpsilonLEDs();
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                  RobotCameras.V3_EPSILON_CAMS);
          break;
        case V3_EPSILON_SIM:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          elevator = new Elevator(new ElevatorIOSim()).getFSM();
          intake = new V3_EpsilonIntake(new V3_EpsilonIntakeIOSim());
          manipulator = new V3_EpsilonManipulator(new V3_EpsilonManipulatorIOSim());
          climber = new V3_EpsilonClimber(new V3_EpsilonClimberIOSim());
          superstructure = new V3_EpsilonSuperstructure(elevator, intake, manipulator);
          leds = new V3_EpsilonLEDs();
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                  RobotCameras.V3_EPSILON_CAMS);
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
      intake = new V3_EpsilonIntake(new V3_EpsilonIntakeIO() {});
    }
    if (manipulator == null) {
      manipulator = new V3_EpsilonManipulator(new V3_EpsilonManipulatorIO() {});
    }
    if (climber == null) {
      climber = new V3_EpsilonClimber(new V3_EpsilonClimberIO() {});
    }
    if (leds == null) {
      leds = new V3_EpsilonLEDs();
    }
    if (superstructure == null) {
      superstructure = new V3_EpsilonSuperstructure(elevator, intake, manipulator);
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
        .onTrue(V3_EpsilonCompositeCommands.setDynamicReefHeight(ReefState.L4, superstructure));
    driver
        .x()
        .and(elevatorNotStow)
        .onTrue(V3_EpsilonCompositeCommands.setDynamicReefHeight(ReefState.L3, superstructure));
    driver
        .b()
        .and(elevatorNotStow)
        .onTrue(V3_EpsilonCompositeCommands.setDynamicReefHeight(ReefState.L2, superstructure));
    driver
        .a()
        .and(elevatorNotStow)
        .onTrue(V3_EpsilonCompositeCommands.setDynamicReefHeight(ReefState.L1, superstructure));

    driver
        .rightTrigger(0.5)
        .whileTrue(
            V3_EpsilonCompositeCommands.optimalAutoScoreCoralSequence(drive, superstructure));
    driver
        .leftTrigger(0.5)
        .whileTrue(
            V3_EpsilonCompositeCommands.intakeCoralDriverSequence(
                superstructure, intake, manipulator))
        .onFalse(
            V3_EpsilonCompositeCommands.postIntakeCoralSequence(
                superstructure, intake, manipulator));

    driver
        .leftBumper()
        .whileTrue(V3_EpsilonCompositeCommands.intakeAlgaeFloor(superstructure, manipulator))
        .onFalse(
            Commands.either(
                superstructure.runGoal(V3_EpsilonSuperstructureStates.STOW_UP),
                superstructure.runGoal(V3_EpsilonSuperstructureStates.STOW_DOWN),
                () -> RobotState.isHasAlgae()));
    driver.rightBumper().onTrue(Commands.runOnce(() -> RobotState.toggleReefPost()));

    driver.povUp().onTrue(superstructure.setPosition());
    driver.povDown().onTrue(SharedCommands.resetHeading(drive));
    driver.povLeft().onTrue(DriveCommands.inchMovement(drive, -0.5, .07));

    driver
        .leftStick()
        .onTrue(V3_EpsilonCompositeCommands.optimalAutoScoreCoralSequence(drive, superstructure));

    driver
        .back()
        .whileTrue(V3_EpsilonCompositeCommands.intakeAlgaeFromReef(drive, superstructure))
        .whileFalse(V3_EpsilonCompositeCommands.postIntakeAlgaeFromReef(drive, superstructure));

    driver
        .start()
        .whileTrue(
            V3_EpsilonCompositeCommands.dropAlgae(
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
        .onTrue(V3_EpsilonCompositeCommands.setDynamicReefHeight(ReefState.L4, superstructure));
    operator
        .x()
        .and(elevatorNotStow)
        .onTrue(V3_EpsilonCompositeCommands.setDynamicReefHeight(ReefState.L3, superstructure));
    operator
        .b()
        .and(elevatorNotStow)
        .onTrue(V3_EpsilonCompositeCommands.setDynamicReefHeight(ReefState.L2, superstructure));
    operator
        .a()
        .and(elevatorNotStow)
        .onTrue(V3_EpsilonCompositeCommands.setDynamicReefHeight(ReefState.L1, superstructure));

    operator
        .leftTrigger(0.5)
        .whileTrue(
            V3_EpsilonCompositeCommands.intakeCoralDriverSequence(
                superstructure, intake, manipulator))
        .whileFalse(
            V3_EpsilonCompositeCommands.postIntakeCoralSequence(
                superstructure, intake, manipulator));
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
        .whileTrue(superstructure.runGoal(V3_EpsilonSuperstructureStates.PROCESSOR))
        .onFalse(
            superstructure
                .runActionWithTimeout(V3_EpsilonSuperstructureStates.PROCESSOR_SCORE, 1)
                .finallyDo(() -> RobotState.setHasAlgae(false)));

    operator.start().whileTrue(superstructure.runGoal(V3_EpsilonSuperstructureStates.BARGE));

    operator.back().onTrue(V3_EpsilonCompositeCommands.optimalScoreBarge(superstructure));
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
        V3_EpsilonMechanism3d.getPoses(
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
    return AutonomousCommands.autoELeft(
            drive, superstructure, intake, manipulator, RobotCameras.V3_EPSILON_CAMS)
        .cmd();
  }
}
