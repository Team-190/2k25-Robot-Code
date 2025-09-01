package frc.robot.subsystems.v3_Epsilon;

// import edu.wpi.first.networktables.NetworkTablesJNI;
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
import frc.robot.subsystems.shared.climber.Climber;
// import frc.robot.subsystems.shared.climber.ClimberIO;
// import frc.robot.subsystems.shared.climber.ClimberIOSim;
import frc.robot.subsystems.shared.climber.ClimberIOTalonFX;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.elevator.ElevatorIO;
import frc.robot.subsystems.shared.elevator.ElevatorIOSim;
import frc.robot.subsystems.shared.elevator.ElevatorIOTalonFX;
// import frc.robot.subsystems.shared.funnel.FunnelIOSim;
// import frc.robot.subsystems.shared.funnel.FunnelIOTalonFX;
// import frc.robot.subsystems.shared.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.shared.vision.VisionConstants.RobotCameras;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructure;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureStates;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIO;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIOSim;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIOTalonFX;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants.ManipulatorRollerStates;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorIO;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorIOSim;
import frc.robot.util.LoggedChoreo.ChoreoChooser;

public class V3_EpsilonRobotContainer implements RobotContainer {
  // Subsystems
  private Climber climber;
  private Drive drive;
  private ElevatorFSM elevator;
  private Vision vision;
  private V3_EpsilonIntake intake;
  // private V3_EpsilonLEDs leds;
  private V3_EpsilonManipulator manipulator;
  private V3_EpsilonSuperstructure superstructure;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Auto chooser
  private final ChoreoChooser autoChooser = new ChoreoChooser();

  public V3_EpsilonRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V3_EPSILON:
          climber = new Climber(new ClimberIOTalonFX());
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0, DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(1, DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(2, DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(3, DriveConstants.BACK_RIGHT));
          elevator = new Elevator(new ElevatorIOTalonFX()).getFSM();
          intake = new V3_EpsilonIntake(new V3_EpsilonIntakeIOTalonFX());
          // leds = new V3_EpsilonLEDs();
          manipulator = new V3_EpsilonManipulator(new V3_EpsilonManipulatorIOSim());
          superstructure =
              new V3_EpsilonSuperstructure(elevator, intake, manipulator); // Doesn't include
          // states
          // vision = new Vision(RobotCameras.V3_Epsilon_CAMS);
          break;
        case V3_EPSILON_SIM:
          // climber = new Climber(new ClimberIOSim());
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          elevator = new Elevator(new ElevatorIOSim()).getFSM();
          // funnel = new Funnel(new FunnelIOSim()).getFSM();
          intake = new V3_EpsilonIntake(new V3_EpsilonIntakeIOSim());
          manipulator = new V3_EpsilonManipulator(new V3_EpsilonManipulatorIOSim());
          // vision = new Vision();
          break;
        default:
          break;
      }
    }

    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {
                // Provide concrete implementation for GyroIO methods here
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              });
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {}).getFSM();
    }
    
    if (intake == null) {
      intake = new V3_EpsilonIntake(new V3_EpsilonIntakeIO() {});
    }
    // if (leds == null) {
    //   leds = new V3_EpsilonLEDs();
    // }
    if (manipulator == null) {
      manipulator = new V3_EpsilonManipulator(new V3_EpsilonManipulatorIO() {});
    }
    // if (vision == null) {
    //   vision = new Vision();
    // }
    superstructure = new V3_EpsilonSuperstructure(elevator, intake, manipulator);

    configureButtonBindings();
    }
}

  public void configureButtonBindings() {
    // Generic triggers
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
    // Trigger halfScoreTrigger =
    // new Trigger(() -> operator.getLeftY() < -DriveConstants.OPERATOR_DEADBAND);
    // Trigger unHalfScoreTrigger =
    // new Trigger(() -> operator.getLeftY() > DriveConstants.OPERATOR_DEADBAND);

    // Trigger operatorFunnelOverride =
    // new Trigger(() -> Math.hypot(operator.getRightX(), operator.getRightY()) >
    // 0.5);

    // Default drive command
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false,
            operator.back(),
            driver.povRight().onTrue(Commands.runOnce(() -> {}))));

    // Driver face buttons
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

    // Driver triggers
    driver
        .leftTrigger(0.5)
        .whileTrue(V3_EpsilonCompositeCommands.intakeCoralDriverSequence(superstructure, intake))
        .onFalse(superstructure.runGoal(V3_EpsilonSuperstructureStates.STOW_DOWN));
    driver
        .rightTrigger(0.5)
        .whileTrue(
            V3_EpsilonCompositeCommands.autoScoreCoralSequence(
                drive, elevator, superstructure, RobotCameras.V3_EPSILON_CAMS));

    // Driver bumpers
    driver
        .leftBumper()
        .whileTrue(Commands.none()); // TODO: Algae ground intake (V3 intake is different)
    driver.rightBumper().onTrue(Commands.runOnce(() -> RobotState.toggleReefPost()));

    // Driver POV
    driver.povUp().onTrue(superstructure.setPosition());
    driver.povDown().onTrue(SharedCommands.resetHeading(drive));
    driver.povLeft().onTrue(DriveCommands.inchMovement(drive, -0.5, .07));

    driver
        .leftStick()
        .onTrue(
            V3_EpsilonCompositeCommands.scoreCoralSequence(
                elevator, superstructure, () -> RobotState.getReefAlignData().atCoralSetpoint()));

    driver
        .back()
        .whileTrue(
            V3_EpsilonCompositeCommands.intakeAlgaeFromReefSequence(
                drive,
                superstructure,
                () -> RobotState.getReefAlignData().algaeIntakeHeight(),
                RobotCameras.V3_EPSILON_CAMS));

    driver
        .start()
        .whileTrue(
            Commands
                .none()); // TODO: add drop algae (V3 drop is going to be significatly different)

    // Operator face buttons
    operator.y().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L4));
    operator.x().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L3));
    operator.b().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L2));
    operator.a().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L1));

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

    // Operator triggers
    operator
        .leftTrigger(0.5)
        .whileTrue(V3_EpsilonCompositeCommands.intakeCoralOperatorSequence(superstructure, intake))
        .onFalse(superstructure.runGoal(V3_EpsilonSuperstructureStates.STOW_DOWN));
    operator
        .rightTrigger(0.5)
        .whileTrue(
            superstructure.override(
                () -> manipulator.setRollerGoal(ManipulatorRollerStates.SCORE_CORAL), 0.4));

    // Operator bumpers
    operator.leftBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)));
    operator.rightBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)));

    operator.povUp().onTrue(V3_EpsilonCompositeCommands.climb(superstructure, climber, drive));
    operator.povDown().whileTrue(climber.winchClimberManual());
    operator
        .povLeft()
        .whileTrue(superstructure.runGoal(V3_EpsilonSuperstructureStates.PROCESSOR_PREP))
        .onFalse(
            superstructure
                .runActionWithTimeout(
                    V3_EpsilonSuperstructureStates.PROCESSOR_PREP,
                    V3_EpsilonSuperstructureStates.PROCESSOR_SCORE,
                    1)
                .finallyDo(() -> RobotState.setHasAlgae(false)));

    operator
        .povRight()
        .whileTrue(
            superstructure
                .override(() -> manipulator.setRollerGoal(ManipulatorRollerStates.SCORE_ALGAE))
                .finallyDo(() -> RobotState.setHasAlgae(false)));
    operator.start().whileTrue(superstructure.runGoal(V3_EpsilonSuperstructureStates.BARGE_PREP));

    operator
        .back()
        .whileTrue(superstructure.runGoal(V3_EpsilonSuperstructureStates.BARGE_PREP))
        .onFalse(
            superstructure
                .runActionWithTimeout(
                    V3_EpsilonSuperstructureStates.BARGE_PREP,
                    V3_EpsilonSuperstructureStates.BARGE_SCORE,
                    0.1)
                .finallyDo(() -> RobotState.setHasAlgae(false)));

    // Misc
    // operatorFunnelOverride
    // .whileTrue(
    // Commands.either(
    // superstructure.runGoal(V3_EpsilonSuperstructureStates.STOW_UP),
    // superstructure.runGoal(
    // V3_EpsilonSuperstructureStates.STOW_DOWN),
    // () -> RobotState.isHasAlgae()))
    // .onFalse(superstructure.runPreviousState());
  }

  private void configureAutos() {
    AutonomousCommands.loadAutoTrajectories(drive);

    autoChooser.addCmd(
        "Drive FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Wheel Radius Characterization", () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addCmd("Elevator Characterization", () -> elevator.sysIdRoutine(superstructure));
    // autoChooser.addCmd("Funnel Characterization", () ->
    // funnel.sysIdRoutine(superstructure));
    autoChooser.addCmd("Intake Characterization", () -> intake.sysIdRoutine(superstructure));
    autoChooser.addCmd(
        "Manipulator Characterization",
        () ->
            Commands.sequence(
                superstructure.runGoal(V3_EpsilonSuperstructureStates.L3_PREP),
                manipulator.sysIdRoutine(superstructure)));
    autoChooser.addRoutine(
        "4 Piece Left",
        () ->
            AutonomousCommands.autoALeft(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));
    autoChooser.addRoutine(
        "4 Piece Right",
        () ->
            AutonomousCommands.autoARight(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));

    autoChooser.addRoutine(
        "4 Piece Left Nashoba",
        () ->
            AutonomousCommands.autoALeftNashoba(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));

    autoChooser.addRoutine(
        "4 Piece Left D.A.V.E.",
        () ->
            AutonomousCommands.autoALeftDAVE(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));

    autoChooser.addRoutine(
        "3 Piece Left",
        () ->
            AutonomousCommands.autoCLeft(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));
    autoChooser.addRoutine(
        "3 Piece Left Push",
        () ->
            AutonomousCommands.autoCLeftPush(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));
    autoChooser.addRoutine(
        "3 Piece Right",
        () ->
            AutonomousCommands.autoCRight(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));
    autoChooser.addRoutine(
        "3 Piece Right Push",
        () ->
            AutonomousCommands.autoCRightPush(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));
    autoChooser.addRoutine(
        "2 Piece Left",
        () ->
            AutonomousCommands.autoBLeft(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));
    autoChooser.addRoutine(
        "2 Piece Right",
        () ->
            AutonomousCommands.autoBRight(
                drive, intake, superstructure, RobotCameras.V3_EPSILON_CAMS));
    autoChooser.addRoutine(
        "1 Piece Center",
        () -> AutonomousCommands.autoDCenter(drive, superstructure, RobotCameras.V3_EPSILON_CAMS));
    SmartDashboard.putData("Autonomous Modes", autoChooser);
  }

  // @Override
  // public void robotPeriodic() {
  // RobotState.periodic(
  // drive.getRawGyroRotation(),
  // NetworkTablesJNI.now(),
  // drive.getYawVelocity(),
  // drive.getFieldRelativeVelocity(),
  // drive.getModulePositions(),
  // manipulator.getArmAngle(),
  // elevator.getPositionMeters());

  // LTNUpdater.updateDrive(drive);
  // LTNUpdater.updateElevator(elevator);
  // LTNUpdater.updateFunnel(funnel);
  // LTNUpdater.updateAlgaeArm(manipulator);
  // LTNUpdater.updateIntake(intake);

  // Logger.recordOutput(
  // "Component Poses",
  // V3_EpsilonMechanism3d.getPoses(
  // elevator.getPositionMeters(),
  // funnel.getAngle(),
  // manipulator.getArmAngle(),
  // intake.getExtension()));
  // }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}

