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
import frc.robot.commands.CompositeCommands.SharedCommands;
import frc.robot.commands.CompositeCommands.SharedCommands.V1_StackUpCompositeCommands;
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
import frc.robot.subsystems.shared.elevator.ElevatorConstants;
import frc.robot.subsystems.shared.elevator.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.shared.elevator.ElevatorIO;
import frc.robot.subsystems.shared.elevator.ElevatorIOSim;
import frc.robot.subsystems.shared.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shared.funnel.Funnel;
import frc.robot.subsystems.shared.funnel.FunnelConstants;
import frc.robot.subsystems.shared.funnel.FunnelConstants.FunnelState;
import frc.robot.subsystems.shared.funnel.FunnelIO;
import frc.robot.subsystems.shared.funnel.FunnelIOSim;
import frc.robot.subsystems.shared.funnel.FunnelIOTalonFX;
import frc.robot.subsystems.shared.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.v1_StackUp.leds.V1_StackUp_LEDs;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulator;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorConstants;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIO;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIOSim;
import frc.robot.subsystems.v1_StackUp.manipulator.V1_StackUpManipulatorIOTalonFX;
import frc.robot.util.KeyboardController;
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
  private final KeyboardController debugBoard = new KeyboardController(0);

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
          vision = new Vision(RobotCameras.V1_STACKUP_CAMS);
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
        .whileTrue(V1_StackUpCompositeCommands.intakeCoral(elevator, funnel, manipulator));
    driver
        .rightTrigger(0.5)
        .whileTrue(
            V1_StackUpCompositeCommands.autoScoreCoralSequence(
                drive, elevator, manipulator, RobotCameras.V1_STACKUP_CAMS));

    // Driver bumpers
    driver.leftBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)));
    driver.rightBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)));

    // Driver algae
    driver.back().onTrue(manipulator.toggleAlgaeArm());
    driver
        .start()
        .onTrue(
            V1_StackUpCompositeCommands.twerk(
                drive, elevator, manipulator, RobotCameras.V1_STACKUP_CAMS));

    // Driver POV
    driver.povUp().onTrue(elevator.setPosition());
    driver
        .povDown()
        .whileTrue(
            Commands.runOnce(() -> RobotState.resetRobotPose(new Pose2d()))
                .alongWith(SharedCommands.resetHeading(drive)));
    driver.povLeft().onTrue(DriveCommands.inchMovement(drive, -0.5, .07));
    driver.povRight().onTrue(DriveCommands.inchMovement(drive, 0.5, .07));
    halfScoreTrigger.whileTrue(manipulator.halfScoreCoral());
    unHalfScoreTrigger.whileTrue((manipulator.unHalfScoreCoral()));

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
        .whileTrue(V1_StackUpCompositeCommands.intakeCoralOverride(elevator, funnel, manipulator));
    operator.rightTrigger(0.5).whileTrue(V1_StackUpCompositeCommands.scoreCoral(manipulator));

    // Operator bumpers
    operator.leftBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)));
    operator.rightBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)));

    operator.povUp().onTrue(SharedCommands.climb(elevator, funnel, climber, drive));
    operator.povDown().whileTrue(climber.winchClimber());

    operator
        .start()
        .onTrue(
            (Commands.runOnce(
                () ->
                    RobotState.resetRobotPose(
                        new Pose2d(0, 0, RobotState.getRobotPoseField().getRotation())))));

    operator.back().whileTrue(V1_StackUpCompositeCommands.emergencyEject(elevator, manipulator));

    // Debug board
    // Base triggers
    debugBoard.resetHeading().onTrue(SharedCommands.resetHeading(drive).ignoringDisable(true));
    // debugBoard
    //     .resetHeadingCameras()
    //     .onTrue(
    //         Commands.runOnce(() -> RobotState.fieldPoseResetMT1(RobotCameras.V1_STACKUP_CAMS))
    //             .ignoringDisable(true));
    debugBoard
        .scoring()
        .primeLeft()
        .onTrue(
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.LEFT)).ignoringDisable(true));
    debugBoard
        .scoring()
        .primeRight()
        .onTrue(
            Commands.runOnce(() -> RobotState.setReefPost(ReefPose.RIGHT)).ignoringDisable(true));
    debugBoard.scoring().track().whileTrue(DriveCommands.autoAlignReefCoral(drive));
    // Funnel triggers
    debugBoard.funnel().wingsClose().onTrue(funnel.setClapDaddyGoal(FunnelState.CLOSED));
    debugBoard.funnel().wingsIntake().onTrue(funnel.setClapDaddyGoal(FunnelState.OPENED));
    debugBoard.funnel().rollerWheelsIn().whileTrue(funnel.setRollerVoltage(12));
    debugBoard.funnel().rollerWheelsOut().whileTrue(funnel.setRollerVoltage(-12));
    debugBoard
        .funnel()
        .incrementRollerWheelsSpeed()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setRollerVoltageOffset(FunnelConstants.ROLLER_OFFSET_INCREMENT_VOLTS)));
    debugBoard
        .funnel()
        .decrementRollerWheelsSpeed()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setRollerVoltageOffset(-FunnelConstants.ROLLER_OFFSET_INCREMENT_VOLTS)));
    debugBoard
        .funnel()
        .incrementClosedSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.CLOSED, -FunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard
        .funnel()
        .incrementIntakeSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.OPENED, -FunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard
        .funnel()
        .decrementClosedSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.CLOSED, FunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard
        .funnel()
        .decrementIntakeSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.OPENED, FunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard.funnel().funnelSensorToggle().onTrue(Commands.runOnce(funnel::toggleSensorOverride));

    // Elevator triggers
    debugBoard
        .elevator()
        .stow()
        .onTrue(SharedCommands.setDynamicReefHeight(ReefHeight.STOW, elevator));
    debugBoard.elevator().raise().onTrue(elevator.setPosition());
    debugBoard.elevator().primeL1().onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L1));
    debugBoard.elevator().primeL2().onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L2));
    debugBoard.elevator().primeL3().onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L3));
    debugBoard.elevator().primeL4().onTrue(SharedCommands.setStaticReefHeight(ReefHeight.L4));
    debugBoard
        .elevator()
        .decreaseL1Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L1,
                        -ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .decreaseL2Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L2,
                        -ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .decreaseL3Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L3,
                        -ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .decreaseL4Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L4,
                        -ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));

    debugBoard
        .elevator()
        .decreaseStowSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.STOW,
                        -ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));

    debugBoard
        .elevator()
        .decreaseAlgaeSetPoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.STOW,
                        -ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));

    debugBoard
        .elevator()
        .increaseL1Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L1,
                        ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseL2Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L2,
                        ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseL3Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L3,
                        ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseL4Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L4,
                        ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseStowSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.STOW,
                        ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));

    debugBoard
        .elevator()
        .increaseAlgaeSetPoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.STOW,
                        ElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    // Manipulator triggers
    debugBoard
        .endEffector()
        .wheelsIn()
        .whileTrue(
            manipulator.runManipulator(V1_StackUpManipulatorConstants.VOLTAGES.HALF_VOLTS().get()));
    debugBoard
        .endEffector()
        .wheelsOut()
        .whileTrue(
            manipulator.runManipulator(
                -V1_StackUpManipulatorConstants.VOLTAGES.HALF_VOLTS().get()));
    debugBoard
        .endEffector()
        .eject()
        .onTrue(V1_StackUpCompositeCommands.emergencyEject(elevator, manipulator));
    debugBoard
        .endEffector()
        .incrementSpeed()
        .onTrue(
            Commands.runOnce(
                () ->
                    manipulator.incrementScoreSpeed(
                        V1_StackUpManipulatorConstants.VOLTAGES.SCORE_OFFSET_INCREMENT().get())));
    debugBoard
        .endEffector()
        .decrementSpeed()
        .onTrue(
            Commands.runOnce(
                () ->
                    manipulator.incrementScoreSpeed(
                        -V1_StackUpManipulatorConstants.VOLTAGES.SCORE_OFFSET_INCREMENT().get())));
    debugBoard.endEffector().toggleSensor().onTrue(manipulator.toggleSensorOverride());
    // Algae triggers
    debugBoard
        .endEffector()
        .blepUp()
        .onTrue(
            V1_StackUpCompositeCommands.twerk(
                drive, elevator, manipulator, ReefHeight.ALGAE_INTAKE_TOP));
    debugBoard
        .endEffector()
        .blepDown()
        .onTrue(
            V1_StackUpCompositeCommands.twerk(
                drive, elevator, manipulator, ReefHeight.ALGAE_INTAKE_BOTTOM));
    debugBoard.endEffector().toggleAss().onTrue(manipulator.toggleAlgaeArm());
    // Climber triggers
    debugBoard.climber().deployLower().onTrue(climber.releaseClimber());
    debugBoard.climber().incrementWintchIn().onTrue(climber.incrementWinchClimber());
    debugBoard.climber().incrementWintchOut().onTrue(climber.decrementWinchClimber());
    debugBoard.climber().sensorOverride().onTrue(climber.manualDeployOverride());
    // // Add climber lane chooser options
    // climberLaneChooser.addDefaultOption("Center", ClimberLane.CENTER);
    // climberLaneChooser.addOption("Right", ClimberLane.RIGHT);
    // climberLaneChooser.addOption("Left", ClimberLane.LEFT);
    // climberLaneChooser
    //     .getSendableChooser()
    //     .onChange(
    //         (l) ->
    //             RobotState.setClimbLane(
    //                 l.equals("Center")
    //                     ? ClimberLane.CENTER
    //                     : l.equals("Right") ? ClimberLane.RIGHT : ClimberLane.LEFT));

    // debugBoard
    //     .climber()
    //     .shiftLaneLeft()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     NetworkTableInstance.getDefault()
    //                         .getTable("SmartDashboard/Climber Lane")
    //                         .getStringTopic("selected")
    //                         .publish()
    //                         .set(
    //                             RobotState.getOIData().climbLane().equals(ClimberLane.CENTER)
    //                                 ? "Left"
    //                                 :
    // RobotState.getOIData().climbLane().equals(ClimberLane.RIGHT)
    //                                     ? "Center"
    //                                     : "Left"))
    //             .ignoringDisable(true));

    // debugBoard
    //     .climber()
    //     .shiftLaneRight()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     NetworkTableInstance.getDefault()
    //                         .getTable("SmartDashboard/Climber Lane")
    //                         .getStringTopic("selected")
    //                         .publish()
    //                         .set(
    //                             RobotState.getOIData().climbLane().equals(ClimberLane.CENTER)
    //                                 ? "Right"
    //                                 : RobotState.getOIData().climbLane().equals(ClimberLane.LEFT)
    //                                     ? "Center"
    //                                     : "Right"))
    //             .ignoringDisable(true));
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
