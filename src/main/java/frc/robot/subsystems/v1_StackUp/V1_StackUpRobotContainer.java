package frc.robot.subsystems.v1_StackUp;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.CompositeCommands.IntakeCommands;
import frc.robot.commands.CompositeCommands.ScoreCommands;
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
import frc.robot.subsystems.shared.funnel.FunnelIO;
import frc.robot.subsystems.shared.funnel.FunnelIOSim;
import frc.robot.subsystems.shared.funnel.FunnelIOTalonFX;
import frc.robot.subsystems.shared.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.v1_StackUp.leds.V1_StackUp_LEDs;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIO;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIOSim;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIOTalonFX;
import frc.robot.util.LTNUpdater;
import org.littletonrobotics.junction.Logger;

public class V1_StackUpRobotContainer implements RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Elevator elevator;
  private Funnel funnel;
  private Climber climber;
  private V1_StackUpManipulator manipulator;
  private V1_StackUp_LEDs leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Auto chooser

  private final AutoChooser autoChooser = new AutoChooser();

  public V1_StackUpRobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V1_STACKUP:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          vision = new Vision(RobotCameras.v1_StackUpCams);
          elevator = new Elevator(new ElevatorIOTalonFX());
          funnel = new Funnel(new FunnelIOTalonFX());
          climber = new Climber(new ClimberIOTalonFX());
          manipulator = new V1_StackUpManipulator(new V1_StackUpManipulatorIOTalonFX());
          leds = new V1_StackUp_LEDs();
          break;
        case V1_STACKUP_SIM:
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
          manipulator = new V1_StackUpManipulator(new V1_StackUpManipulatorIOSim());
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
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (funnel == null) {
      funnel = new Funnel(new FunnelIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (manipulator == null) {
      manipulator = new V1_StackUpManipulator(new V1_StackUpManipulatorIO() {});
    }
    if (leds == null) {
      leds = new V1_StackUp_LEDs();
    }

    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {
    // Generic triggers
    Trigger elevatorStow =
        new Trigger(
            () ->
                elevator.getPosition().equals(ElevatorPositions.INTAKE)
                    || elevator.getPosition().equals(ElevatorPositions.STOW));
    Trigger elevatorNotStow =
        new Trigger(
            () ->
                !elevator.getPosition().equals(ElevatorPositions.INTAKE)
                    && !elevator.getPosition().equals(ElevatorPositions.STOW));
    Trigger halfScoreTrigger =
        new Trigger(() -> operator.getLeftY() < -DriveConstants.OPERATOR_DEADBAND);
    Trigger unHalfScoreTrigger =
        new Trigger(() -> operator.getLeftY() > DriveConstants.OPERATOR_DEADBAND);

    // Default drive command
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false));

    // Driver face buttons
    driver.y().and(elevatorStow).onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L4));
    driver.x().and(elevatorStow).onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L3));
    driver.b().and(elevatorStow).onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L2));
    driver.a().and(elevatorStow).onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L1));

    driver
        .y()
        .and(elevatorNotStow)
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.L4, elevator));
    driver
        .x()
        .and(elevatorNotStow)
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.L3, elevator));
    driver
        .b()
        .and(elevatorNotStow)
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.L2, elevator));
    driver
        .a()
        .and(elevatorNotStow)
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.L1, elevator));

    // Driver triggers
    driver.leftTrigger(0.5).whileTrue(IntakeCommands.intakeCoral(elevator, funnel, manipulator));
    driver
        .rightTrigger(0.5)
        .whileTrue(
            ScoreCommands.autoScoreCoralSequence(
                drive, elevator, manipulator, RobotCameras.v1_StackUpCams));

    // Driver bumpers
    driver.leftBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)));
    driver.rightBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)));

    // Driver algae
    driver.back().onTrue(manipulator.toggleAlgaeArm());
    driver
        .start()
        .onTrue(ScoreCommands.twerk(drive, elevator, manipulator, RobotCameras.v1_StackUpCams));

    // Driver POV
    driver.povUp().onTrue(elevator.setPosition());
    driver
        .povDown()
        .whileTrue(
            Commands.runOnce(() -> RobotState.resetRobotPose(new Pose2d()))
                .alongWith(CompositeCommands.resetHeading(drive)));
    driver.povLeft().onTrue(DriveCommands.inchMovement(drive, -0.5, .07));
    driver.povRight().onTrue(DriveCommands.inchMovement(drive, 0.5, .07));
    halfScoreTrigger.whileTrue(manipulator.halfScoreCoral());
    unHalfScoreTrigger.whileTrue((manipulator.unHalfScoreCoral()));

    // Operator face buttons
    operator.y().and(elevatorStow).onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L4));
    operator.x().and(elevatorStow).onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L3));
    operator.b().and(elevatorStow).onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L2));
    operator.a().and(elevatorStow).onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L1));

    operator
        .y()
        .and(elevatorNotStow)
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.L4, elevator));
    operator
        .x()
        .and(elevatorNotStow)
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.L3, elevator));
    operator
        .b()
        .and(elevatorNotStow)
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.L2, elevator));
    operator
        .a()
        .and(elevatorNotStow)
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.L1, elevator));

    // Operator triggers
    operator
        .leftTrigger(0.5)
        .whileTrue(IntakeCommands.intakeCoralOverride(elevator, funnel, manipulator));
    operator.rightTrigger(0.5).whileTrue(ScoreCommands.scoreCoral(manipulator));

    // Operator bumpers
    operator.leftBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)));
    operator.rightBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)));

    operator.povUp().onTrue(CompositeCommands.climb(elevator, funnel, climber, drive));
    operator.povDown().whileTrue(climber.winchClimber());

    operator
        .start()
        .onTrue(
            (Commands.runOnce(
                () ->
                    RobotState.resetRobotPose(
                        new Pose2d(0, 0, RobotState.getRobotPoseField().getRotation())))));

    operator.back().whileTrue(ScoreCommands.emergencyEject(elevator, manipulator));
  }

  private void configureAutos() {
    AutonomousCommands.loadAutoTrajectories(drive);

    autoChooser.addCmd("None", Commands::none);
    autoChooser.addCmd(
        "Drive FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Wheel Radius Characterization", () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "4 Piece Left",
        () ->
            AutonomousCommands.autoALeft(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams));
    autoChooser.addCmd(
        "4 Piece Right",
        () ->
            AutonomousCommands.autoARight(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams));
    autoChooser.addCmd(
        "3 Piece Left",
        () ->
            AutonomousCommands.autoCLeft(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams));
    autoChooser.addCmd(
        "3 Piece Right",
        () ->
            AutonomousCommands.autoCRight(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams));
    autoChooser.addCmd(
        "2 Piece Left",
        () ->
            AutonomousCommands.autoBLeft(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams));
    autoChooser.addCmd(
        "2 Piece Right",
        () ->
            AutonomousCommands.autoBRight(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams));
    autoChooser.addCmd(
        "1 Piece Center",
        () ->
            AutonomousCommands.autoDCenter(
                drive, elevator, manipulator, RobotCameras.v1_StackUpCams));
    SmartDashboard.putData("Autonomous Mode", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getFieldRelativeVelocity(),
        drive.getModulePositions(),
        vision.getCameras());

    LTNUpdater.updateDrive(drive);
    LTNUpdater.updateElevator(elevator);
    LTNUpdater.updateFunnel(funnel);

    if (Constants.getMode().equals(Mode.SIM)) {
      Logger.recordOutput(
          "Component Poses",
          V1_StackUpMechanism3d.getPoses(elevator.getPositionMeters(), funnel.getAngle()));
    }
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
