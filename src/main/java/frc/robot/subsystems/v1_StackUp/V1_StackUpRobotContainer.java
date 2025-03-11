package frc.robot.subsystems.v1_StackUp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
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
import frc.robot.commands.DriveCommands.ClimberLane;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.v1_StackUp.climber.V1_StackUpClimber;
import frc.robot.subsystems.v1_StackUp.climber.V1_StackUpClimberIO;
import frc.robot.subsystems.v1_StackUp.climber.V1_StackUpClimberIOSim;
import frc.robot.subsystems.v1_StackUp.climber.V1_StackUpClimberIOTalonFX;
import frc.robot.subsystems.v1_StackUp.elevator.V1_StackUpElevator;
import frc.robot.subsystems.v1_StackUp.elevator.V1_StackUpElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.v1_StackUp.elevator.V1_StackUpElevatorIO;
import frc.robot.subsystems.v1_StackUp.elevator.V1_StackUpElevatorIOSim;
import frc.robot.subsystems.v1_StackUp.elevator.V1_StackUpElevatorIOTalonFX;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnel;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnelIO;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnelIOSim;
import frc.robot.subsystems.v1_StackUp.funnel.V1_StackUpFunnelIOTalonFX;
import frc.robot.subsystems.v1_StackUp.leds.V1_StackUp_LEDs;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIO;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIOSim;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIOTalonFX;
import frc.robot.util.LTNUpdater;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class V1_StackUpRobotContainer implements RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  private V1_StackUpElevator elevator;
  private V1_StackUpFunnel funnel;
  private V1_StackUpManipulator manipulator;
  private V1_StackUpClimber climber;

  private V1_StackUp_LEDs leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Modes");

  private final LoggedDashboardChooser<ClimberLane> climberLaneChooser =
      new LoggedDashboardChooser<>("Climber Lane");

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
          elevator = new V1_StackUpElevator(new V1_StackUpElevatorIOTalonFX());
          funnel = new V1_StackUpFunnel(new V1_StackUpFunnelIOTalonFX());
          manipulator = new V1_StackUpManipulator(new V1_StackUpManipulatorIOTalonFX());
          climber = new V1_StackUpClimber(new V1_StackUpClimberIOTalonFX());
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
          elevator = new V1_StackUpElevator(new V1_StackUpElevatorIOSim());
          funnel = new V1_StackUpFunnel(new V1_StackUpFunnelIOSim());
          manipulator = new V1_StackUpManipulator(new V1_StackUpManipulatorIOSim());
          climber = new V1_StackUpClimber(new V1_StackUpClimberIOSim());
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
      elevator = new V1_StackUpElevator(new V1_StackUpElevatorIO() {});
    }
    if (funnel == null) {
      funnel = new V1_StackUpFunnel(new V1_StackUpFunnelIO() {});
    }
    if (manipulator == null) {
      manipulator = new V1_StackUpManipulator(new V1_StackUpManipulatorIO() {});
    }
    if (leds == null) {
      leds = new V1_StackUp_LEDs();
    }
    if (climber == null) {
      climber = new V1_StackUpClimber(new V1_StackUpClimberIO() {});
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
            driver.getHID()::getBButton,
            driver.leftTrigger(0.5),
            () -> driver.getHID().getPOV() == 90));

    // Driver face buttons
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
    driver.x().onTrue(elevator.setPosition());
    driver.b().onTrue(elevator.setPosition());
    driver.a().whileTrue(elevator.setPosition(ReefHeight.STOW));

    // Driver triggers
    driver.leftTrigger(0.5).whileTrue(IntakeCommands.intakeCoral(elevator, funnel, manipulator));
    driver
        .rightTrigger(0.5)
        .whileTrue(
            ScoreCommands.autoScoreCoralSequence(
                drive, elevator, manipulator, RobotCameras.v1_StackUpCams));

    // Driver bumpers
    driver.leftBumper().onTrue(DriveCommands.inchMovement(drive, -0.5, .07));
    driver.rightBumper().onTrue(DriveCommands.inchMovement(drive, 0.5, .07));

    driver.back().onTrue(manipulator.toggleAlgaeArm());
    driver.start().onTrue(ScoreCommands.twerk(drive, elevator, manipulator));

    driver
        .povUp()
        .onTrue(
            Commands.runOnce(
                () ->
                    RobotState.resetRobotPose(
                        new Pose2d(
                            new Translation2d(), RobotState.getRobotPoseReef().getRotation()))));

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
        .or(operator.back())
        .whileTrue(ScoreCommands.emergencyEject(elevator, manipulator));

    // Add climber lane chooser options
    climberLaneChooser.addDefaultOption("Center", ClimberLane.CENTER);
    climberLaneChooser.addOption("Right", ClimberLane.RIGHT);
    climberLaneChooser.addOption("Left", ClimberLane.LEFT);
    climberLaneChooser
        .getSendableChooser()
        .onChange(
            (l) ->
                RobotState.setClimbLane(
                    l.equals("Center")
                        ? ClimberLane.CENTER
                        : l.equals("Right") ? ClimberLane.RIGHT : ClimberLane.LEFT));
  }

  private void configureAutos() {
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption(
        "Drive FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "4 Piece Left",
        AutonomousCommands.autoALeft(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams)
            .cmd());
    autoChooser.addOption(
        "4 Piece Right",
        AutonomousCommands.autoARight(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams)
            .cmd());
    autoChooser.addOption(
        "3 Piece Left",
        AutonomousCommands.autoCLeft(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams)
            .cmd());
    autoChooser.addOption(
        "3 Piece Right",
        AutonomousCommands.autoCRight(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams)
            .cmd());
    autoChooser.addOption(
        "2 Piece Left",
        AutonomousCommands.autoBLeft(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams)
            .cmd());
    autoChooser.addOption(
        "2 Piece Right",
        AutonomousCommands.autoBRight(
                drive, elevator, funnel, manipulator, RobotCameras.v1_StackUpCams)
            .cmd());
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
    return autoChooser.get();
  }
}
