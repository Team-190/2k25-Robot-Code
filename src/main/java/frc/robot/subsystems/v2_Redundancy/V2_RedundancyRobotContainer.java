package frc.robot.subsystems.v2_Redundancy;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.CompositeCommands.SharedCommands;
import frc.robot.commands.CompositeCommands.SharedCommands.V2_RedundancyCompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.climber.Climber;
import frc.robot.subsystems.shared.climber.ClimberIO;
import frc.robot.subsystems.shared.climber.ClimberIOSim;
import frc.robot.subsystems.shared.climber.ClimberIOTalonFX;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.elevator.ElevatorIO;
import frc.robot.subsystems.shared.elevator.ElevatorIOSim;
import frc.robot.subsystems.shared.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shared.funnel.Funnel;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.shared.funnel.FunnelIO;
import frc.robot.subsystems.shared.funnel.FunnelIOSim;
import frc.robot.subsystems.shared.funnel.FunnelIOTalonFX;
import frc.robot.subsystems.shared.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntakeIO;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntakeIOSim;
import frc.robot.subsystems.v2_Redundancy.intake.V2_RedundancyIntakeIOTalonFX;
import frc.robot.subsystems.v2_Redundancy.leds.V2_RedundancyLEDs;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulatorIO;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulatorIOSim;
import frc.robot.subsystems.v2_Redundancy.manipulator.V2_RedundancyManipulatorIOTalonFX;
import frc.robot.util.LTNUpdater;
import frc.robot.util.LoggedChoreo.ChoreoChooser;
import org.littletonrobotics.junction.Logger;

public class V2_RedundancyRobotContainer implements RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Elevator elevator;
  private Funnel funnel;
  private Climber climber;
  private V2_RedundancyManipulator manipulator;
  private V2_RedundancyIntake intake;
  private V2_RedundancyLEDs leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Auto chooser
  private final ChoreoChooser autoChooser = new ChoreoChooser();

  public V2_RedundancyRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V2_REDUNDANCY:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(0, DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(1, DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(2, DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(3, DriveConstants.BACK_RIGHT));
          vision = new Vision(RobotCameras.V2_REDUNDANCY_CAMS);
          elevator = new Elevator(new ElevatorIOTalonFX());
          funnel = new Funnel(new FunnelIOTalonFX());
          climber = new Climber(new ClimberIOTalonFX());
          manipulator = new V2_RedundancyManipulator(new V2_RedundancyManipulatorIOTalonFX());
          intake = new V2_RedundancyIntake(new V2_RedundancyIntakeIOTalonFX());
          leds = new V2_RedundancyLEDs();
          break;
        case V2_REDUNDANCY_SIM:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          vision = new Vision();
          elevator = new Elevator(new ElevatorIOSim());
          funnel = new Funnel(new FunnelIOSim());
          climber = new Climber(new ClimberIOSim());
          manipulator = new V2_RedundancyManipulator(new V2_RedundancyManipulatorIOSim());
          intake = new V2_RedundancyIntake(new V2_RedundancyIntakeIOSim());
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
    if (vision == null) {
      vision = new Vision();
    }
    if (funnel == null) {
      funnel = new Funnel(new FunnelIO() {});
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (manipulator == null) {
      manipulator = new V2_RedundancyManipulator(new V2_RedundancyManipulatorIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (intake == null) {
      intake = new V2_RedundancyIntake(new V2_RedundancyIntakeIO() {});
    }
    if (leds == null) {
      leds = new V2_RedundancyLEDs();
    }

    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {
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
    //     new Trigger(() -> operator.getLeftY() < -DriveConstants.OPERATOR_DEADBAND);
    // Trigger unHalfScoreTrigger =
    //     new Trigger(() -> operator.getLeftY() > DriveConstants.OPERATOR_DEADBAND);

    Trigger operatorFunnelOverride =
        new Trigger(() -> Math.hypot(operator.getRightX(), operator.getRightY()) > 0.5);

    // Default drive command
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false,
            () -> false));

    // Driver face buttons
    driver.y().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L4));
    driver.x().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L3));
    driver.b().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L2));
    driver.a().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L1));

    driver
        .y()
        .and(elevatorNotStow)
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.L4, elevator));
    driver
        .x()
        .and(elevatorNotStow)
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.L3, elevator));
    driver
        .b()
        .and(elevatorNotStow)
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.L2, elevator));
    driver
        .a()
        .and(elevatorNotStow)
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.L1, elevator));

    // Driver triggers
    driver
        .leftTrigger(0.5)
        .whileTrue(
            V2_RedundancyCompositeCommands.intakeCoralDriverSequence(
                elevator, funnel, manipulator, intake));
    driver
        .rightTrigger(0.5)
        .whileTrue(
            V2_RedundancyCompositeCommands.autoScoreCoralSequence(
                drive, elevator, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));

    // Driver bumpers
    driver
        .leftBumper()
        .whileTrue(
            V2_RedundancyCompositeCommands.floorIntakeSequence(manipulator, elevator, intake))
        .onFalse(
            V2_RedundancyCompositeCommands.postFloorIntakeSequence(manipulator, elevator, intake));
    driver.rightBumper().onTrue(Commands.runOnce(() -> RobotState.toggleReefPost()));

    // Driver POV
    driver.povUp().onTrue(elevator.setPosition());
    driver.povDown().onTrue(SharedCommands.resetHeading(drive));
    driver.povLeft().onTrue(DriveCommands.inchMovement(drive, -0.5, .07));
    driver.povRight().onTrue(DriveCommands.inchMovement(drive, 0.5, .07));

    driver
        .leftStick()
        .onTrue(
            V2_RedundancyCompositeCommands.scoreCoralSequence(
                elevator,
                manipulator,
                intake,
                () -> RobotState.getReefAlignData().atCoralSetpoint()));

    driver
        .start()
        .whileTrue(
            V2_RedundancyCompositeCommands.intakeAlgaeFromReefSequence(
                drive,
                elevator,
                manipulator,
                intake,
                () -> RobotState.getReefAlignData().algaeIntakeHeight(),
                RobotCameras.V2_REDUNDANCY_CAMS));
    driver
        .back()
        .whileTrue(
            V2_RedundancyCompositeCommands.dropAlgae(
                drive,
                elevator,
                manipulator,
                intake,
                () -> RobotState.getReefAlignData().algaeIntakeHeight()));

    // Operator face buttons
    operator.y().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L4));
    operator.x().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L3));
    operator.b().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L2));
    operator.a().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L1));

    operator
        .y()
        .and(elevatorNotStow)
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.L4, elevator));
    operator
        .x()
        .and(elevatorNotStow)
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.L3, elevator));
    operator
        .b()
        .and(elevatorNotStow)
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.L2, elevator));
    operator
        .a()
        .and(elevatorNotStow)
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.L1, elevator));

    // Operator triggers
    operator
        .leftTrigger(0.5)
        .whileTrue(
            V2_RedundancyCompositeCommands.intakeCoralOperatorSequence(
                elevator, funnel, manipulator, intake))
        .onFalse(
            Commands.parallel(funnel.setClapDaddyGoal(FunnelState.OPENED), funnel.stopRoller()));
    operator.rightTrigger(0.5).whileTrue(V2_RedundancyCompositeCommands.scoreCoral(manipulator));

    // Operator bumpers
    operator.leftBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)));
    operator.rightBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)));

    operator.povUp().onTrue(SharedCommands.climb(elevator, funnel, climber, drive));
    operator.povDown().whileTrue(climber.winchClimberManual());
    operator
        .povLeft()
        .whileTrue(V2_RedundancyCompositeCommands.scoreProcessor(elevator, manipulator, intake))
        .onFalse(manipulator.scoreAlgae().withTimeout(1));

    operator.povRight().whileTrue(manipulator.scoreAlgae());
    operator
        .back()
        .whileTrue(V2_RedundancyCompositeCommands.netHeight(elevator, manipulator, intake));

    // Misc
    operatorFunnelOverride.whileTrue(
        V2_RedundancyCompositeCommands.intakeCoralOperatorOverrideSequence(
            elevator, funnel, manipulator, intake));
    operatorFunnelOverride.onFalse(
        Commands.sequence(funnel.setClapDaddyGoal(FunnelState.OPENED), funnel.stopRoller()));
  }

  private void configureAutos() {
    AutonomousCommands.loadAutoTrajectories(drive);

    autoChooser.addCmd("None", Commands::none);
    autoChooser.addCmd(
        "Drive FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Wheel Radius Characterization", () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addRoutine(
        "4 Piece Left",
        () ->
            AutonomousCommands.autoALeft(
                drive, elevator, funnel, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));
    autoChooser.addCmd(
        "4 Piece Right",
        () ->
            AutonomousCommands.autoARight(
                drive, elevator, funnel, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));
    autoChooser.addCmd(
        "3 Piece Left",
        () ->
            AutonomousCommands.autoCLeft(
                drive, elevator, funnel, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));
    autoChooser.addCmd(
        "3 Piece Left Push",
        () ->
            AutonomousCommands.autoCLeftPush(
                drive, elevator, funnel, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));
    autoChooser.addCmd(
        "3 Piece Right",
        () ->
            AutonomousCommands.autoCRight(
                drive, elevator, funnel, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));
    autoChooser.addCmd(
        "3 Piece Right Push",
        () ->
            AutonomousCommands.autoCRightPush(
                drive, elevator, funnel, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));
    autoChooser.addCmd(
        "2 Piece Left",
        () ->
            AutonomousCommands.autoBLeft(
                drive, elevator, funnel, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));
    autoChooser.addCmd(
        "2 Piece Right",
        () ->
            AutonomousCommands.autoBRight(
                drive, elevator, funnel, manipulator, intake, RobotCameras.V2_REDUNDANCY_CAMS));
    autoChooser.addCmd(
        "1 Piece Center",
        () ->
            AutonomousCommands.autoDCenter(
                drive, elevator, manipulator, RobotCameras.V2_REDUNDANCY_CAMS));
    SmartDashboard.putData("Autonomous Modes", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getFieldRelativeVelocity(),
        drive.getModulePositions(),
        intake.getExtension(),
        manipulator.getArmAngle(),
        elevator.getPositionMeters(),
        vision.getCameras());

    LTNUpdater.updateDrive(drive);
    LTNUpdater.updateElevator(elevator);
    LTNUpdater.updateFunnel(funnel);
    LTNUpdater.updateAlgaeArm(manipulator);
    LTNUpdater.updateIntake(intake);

    Logger.recordOutput(
        "Component Poses",
        V2_RedundancyMechanism3d.getPoses(
            elevator.getPositionMeters(),
            funnel.getAngle(),
            manipulator.getArmAngle(),
            intake.getExtension()));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
