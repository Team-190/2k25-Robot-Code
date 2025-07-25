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
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.RobotContainer;
import frc.robot.RobotStateLL;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.CompositeCommands.SharedCommands;
import frc.robot.commands.CompositeCommands.V1_StackUpCompositeCommands;
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
import frc.robot.subsystems.shared.elevator.ElevatorCSB;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.elevator.ElevatorIO;
import frc.robot.subsystems.shared.elevator.ElevatorIOSim;
import frc.robot.subsystems.shared.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shared.visionlimelight.CameraConstants.RobotCameras;
import frc.robot.subsystems.shared.visionlimelight.Vision;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnelCSB;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnelIO;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnelIOSim;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnelIOTalonFX;
import frc.robot.subsystems.v1_StackUp.leds.V1_StackUpLEDs;
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
  private ElevatorCSB elevator;
  private V1_StackUpFunnelCSB funnel;
  private Climber climber;
  private V1_StackUpManipulator manipulator;
  private V1_StackUpLEDs leds;

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
                  new ModuleIOTalonFX(0, DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(1, DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(2, DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(3, DriveConstants.BACK_RIGHT));
          vision = new Vision(RobotCameras.V1_STACKUP_CAMS);
          elevator = new ElevatorCSB(new ElevatorIOTalonFX());
          funnel = new V1_StackUpFunnelCSB(new V1_StackUpFunnelIOTalonFX());
          climber = new Climber(new ClimberIOTalonFX());
          manipulator = new V1_StackUpManipulator(new V1_StackUpManipulatorIOTalonFX());
          leds = new V1_StackUpLEDs();
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
          elevator = new ElevatorCSB(new ElevatorIOSim());
          funnel = new V1_StackUpFunnelCSB(new V1_StackUpFunnelIOSim());
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
      elevator = new ElevatorCSB(new ElevatorIO() {});
    }
    if (funnel == null) {
      funnel = new V1_StackUpFunnelCSB(new V1_StackUpFunnelIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (manipulator == null) {
      manipulator = new V1_StackUpManipulator(new V1_StackUpManipulatorIO() {});
    }
    if (leds == null) {
      leds = new V1_StackUpLEDs();
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
            () -> false,
            driver.back(),
            () -> false));

    // Driver face buttons
    driver.y().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L4));
    driver.x().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L3));
    driver.b().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L2));
    driver.a().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L1));

    driver
        .y()
        .and(elevatorNotStow)
        .onTrue(V1_StackUpCompositeCommands.setDynamicReefHeight(ReefState.L4, elevator));
    driver
        .x()
        .and(elevatorNotStow)
        .onTrue(V1_StackUpCompositeCommands.setDynamicReefHeight(ReefState.L3, elevator));
    driver
        .b()
        .and(elevatorNotStow)
        .onTrue(V1_StackUpCompositeCommands.setDynamicReefHeight(ReefState.L2, elevator));
    driver
        .a()
        .and(elevatorNotStow)
        .onTrue(V1_StackUpCompositeCommands.setDynamicReefHeight(ReefState.L1, elevator));

    // Driver triggers
    driver
        .leftTrigger(0.5)
        .whileTrue(V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator));
    driver
        .rightTrigger(0.5)
        .whileTrue(
            V1_StackUpCompositeCommands.autoScoreCoralSequence(
                drive, elevator, manipulator, RobotCameras.V1_STACKUP_CAMS));

    // Driver bumpers
    driver.leftBumper().onTrue(Commands.runOnce(() -> RobotStateLL.setReefPost(ReefPose.LEFT)));
    driver.rightBumper().onTrue(Commands.runOnce(() -> RobotStateLL.setReefPost(ReefPose.RIGHT)));

    // Driver algae
    /*driver.back().onTrue(manipulator.toggleAlgaeArm());
    driver
        .start()
        .onTrue(AlgaeCommands.twerk(drive, elevator, manipulator, RobotCameras.V1_STACKUP_CAMS));*/

    // Driver POV
    driver.povUp().onTrue(elevator.setPosition());
    driver
        .povDown()
        .whileTrue(
            Commands.runOnce(() -> RobotStateLL.resetRobotPose(new Pose2d()))
                .alongWith(SharedCommands.resetHeading(drive)));
    driver.povLeft().onTrue(DriveCommands.inchMovement(drive, -0.5, .07));
    driver.povRight().onTrue(DriveCommands.inchMovement(drive, 0.5, .07));
    halfScoreTrigger.whileTrue(manipulator.halfScoreCoral());
    unHalfScoreTrigger.whileTrue((manipulator.unHalfScoreCoral()));

    // Operator face buttons
    operator.y().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L4));
    operator.x().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L3));
    operator.b().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L2));
    operator.a().and(elevatorStow).onTrue(SharedCommands.setStaticReefHeight(ReefState.L1));

    operator
        .y()
        .and(elevatorNotStow)
        .onTrue(V1_StackUpCompositeCommands.setDynamicReefHeight(ReefState.L4, elevator));
    operator
        .x()
        .and(elevatorNotStow)
        .onTrue(V1_StackUpCompositeCommands.setDynamicReefHeight(ReefState.L3, elevator));
    operator
        .b()
        .and(elevatorNotStow)
        .onTrue(V1_StackUpCompositeCommands.setDynamicReefHeight(ReefState.L2, elevator));
    operator
        .a()
        .and(elevatorNotStow)
        .onTrue(V1_StackUpCompositeCommands.setDynamicReefHeight(ReefState.L1, elevator));

    // Operator triggers
    operator
        .leftTrigger(0.5)
        .whileTrue(V1_StackUpCompositeCommands.intakeCoralOverride(elevator, funnel, manipulator));
    operator.rightTrigger(0.5).whileTrue(V1_StackUpCompositeCommands.scoreCoral(manipulator));

    // Operator bumpers
    operator.leftBumper().onTrue(Commands.runOnce(() -> RobotStateLL.setReefPost(ReefPose.LEFT)));
    operator.rightBumper().onTrue(Commands.runOnce(() -> RobotStateLL.setReefPost(ReefPose.RIGHT)));

    operator.povUp().onTrue(V1_StackUpCompositeCommands.climb(elevator, funnel, climber, drive));
    operator.povDown().whileTrue(climber.winchClimber());

    operator
        .start()
        .onTrue(
            (Commands.runOnce(
                () ->
                    RobotStateLL.resetRobotPose(
                        new Pose2d(0, 0, RobotStateLL.getRobotPoseField().getRotation())))));

    operator.back().whileTrue(V1_StackUpCompositeCommands.emergencyEject(elevator, manipulator));
  }

  private void configureAutos() {
    AutonomousCommands.loadAutoTrajectoriesOld(drive);

    autoChooser.addCmd("None", Commands::none);
    autoChooser.addCmd(
        "Drive FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Wheel Radius Characterization", () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "4 Piece Left",
        () ->
            AutonomousCommands.autoALeft(
                drive, elevator, funnel, manipulator, RobotCameras.V1_STACKUP_CAMS));
    autoChooser.addCmd(
        "4 Piece Right",
        () ->
            AutonomousCommands.autoARight(
                drive, elevator, funnel, manipulator, RobotCameras.V1_STACKUP_CAMS));
    autoChooser.addCmd(
        "3 Piece Left",
        () ->
            AutonomousCommands.autoCLeft(
                drive, elevator, funnel, manipulator, RobotCameras.V1_STACKUP_CAMS));
    autoChooser.addCmd(
        "3 Piece Right",
        () ->
            AutonomousCommands.autoCRight(
                drive, elevator, funnel, manipulator, RobotCameras.V1_STACKUP_CAMS));
    autoChooser.addCmd(
        "2 Piece Left",
        () ->
            AutonomousCommands.autoBLeft(
                drive, elevator, funnel, manipulator, RobotCameras.V1_STACKUP_CAMS));
    autoChooser.addCmd(
        "2 Piece Right",
        () ->
            AutonomousCommands.autoBRight(
                drive, elevator, funnel, manipulator, RobotCameras.V1_STACKUP_CAMS));
    autoChooser.addCmd(
        "1 Piece Center",
        () ->
            AutonomousCommands.autoDCenter(
                drive, elevator, manipulator, RobotCameras.V1_STACKUP_CAMS));
    SmartDashboard.putData("Autonomous Modes", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    RobotStateLL.periodic(
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
