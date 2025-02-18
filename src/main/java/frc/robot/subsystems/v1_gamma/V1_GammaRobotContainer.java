package frc.robot.subsystems.v1_gamma;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants.Reef.ReefHeight;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.CompositeCommands.IntakeCommands;
import frc.robot.commands.CompositeCommands.ScoreCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.vision.CameraConstants.RobotCameras;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.v1_gamma.climber.V1_GammaClimber;
import frc.robot.subsystems.v1_gamma.climber.V1_GammaClimberIO;
import frc.robot.subsystems.v1_gamma.climber.V1_GammaClimberIOSim;
import frc.robot.subsystems.v1_gamma.climber.V1_GammaClimberIOTalonFX;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevator;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorConstants;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorIO;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorIOSim;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorIOTalonFX;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnel;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelConstants;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelConstants.FunnelState;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelIO;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelIOSim;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelIOTalonFX;
import frc.robot.subsystems.v1_gamma.leds.V1_Gamma_LEDs;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulator;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulatorConstants;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulatorIO;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulatorIOSim;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulatorIOTalonFX;
import frc.robot.util.KeyboardController;
import frc.robot.util.LTNUpdater;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class V1_GammaRobotContainer implements RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;

  private V1_GammaElevator elevator;
  private V1_GammaFunnel funnel;
  private V1_GammaManipulator manipulator;
  private V1_GammaClimber climber;

  private V1_Gamma_LEDs leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final KeyboardController debugBoard = new KeyboardController(0);

  // Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Modes");

  public V1_GammaRobotContainer() {

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
          vision = new Vision(RobotCameras.v1_GammaCams);
          elevator = new V1_GammaElevator(new V1_GammaElevatorIOTalonFX());
          funnel = new V1_GammaFunnel(new V1_GammaFunnelIOTalonFX());
          manipulator = new V1_GammaManipulator(new V1_GammaManipulatorIOTalonFX());
          climber = new V1_GammaClimber(new V1_GammaClimberIOTalonFX());
          leds = new V1_Gamma_LEDs();
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
          elevator = new V1_GammaElevator(new V1_GammaElevatorIOSim());
          funnel = new V1_GammaFunnel(new V1_GammaFunnelIOSim());
          manipulator = new V1_GammaManipulator(new V1_GammaManipulatorIOSim());
          climber = new V1_GammaClimber(new V1_GammaClimberIOSim());
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
      elevator = new V1_GammaElevator(new V1_GammaElevatorIO() {});
    }
    if (funnel == null) {
      funnel = new V1_GammaFunnel(new V1_GammaFunnelIO() {});
    }
    if (manipulator == null) {
      manipulator = new V1_GammaManipulator(new V1_GammaManipulatorIO() {});
    }
    if (leds == null) {
      leds = new V1_Gamma_LEDs();
    }
    if (climber == null) {
      climber = new V1_GammaClimber(new V1_GammaClimberIO() {});
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
            driver.getHID()::getBButton));

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
                drive,
                elevator,
                funnel,
                manipulator,
                RobotState.getOperatorInputData().currentReefHeight(),
                RobotCameras.v1_GammaCams));

    // Driver bumpers
    driver.leftBumper().onTrue(DriveCommands.inchMovement(drive, -0.5));
    driver.rightBumper().onTrue(DriveCommands.inchMovement(drive, 0.5));

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
    operator.leftTrigger(0.5).whileTrue(IntakeCommands.intakeCoral(elevator, funnel, manipulator));
    operator.rightTrigger(0.5).whileTrue(ScoreCommands.scoreCoral(elevator, manipulator));

    // Operator bumpers
    operator.leftBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)));
    operator.rightBumper().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)));

    operator.povUp().onTrue(CompositeCommands.climb(elevator, funnel, climber));
    operator.povDown().whileTrue(climber.winchClimber());

    // Debug board
    // Base triggers
    debugBoard.resetHeading().onTrue(CompositeCommands.resetHeading(drive));
    // TODO: Add Translate & rotate commands
    debugBoard
        .scoring()
        .primeLeft()
        .onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)));
    debugBoard
        .scoring()
        .primeRight()
        .onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)));
    debugBoard.scoring().track().whileTrue(DriveCommands.alignRobotToAprilTag(drive));
    // Funnel triggers
    debugBoard.funnel().wingsClose().onTrue(funnel.setClapDaddyGoal(FunnelState.CLOSED));
    debugBoard.funnel().wingsIntake().onTrue(funnel.setClapDaddyGoal(FunnelState.OPENED));
    debugBoard.funnel().wingsStingerOut().onTrue(funnel.setClapDaddyGoal(FunnelState.CLIMB));
    debugBoard
        .funnel()
        .wheelsIn()
        .whileTrue(
            funnel.setRollerVoltage(
                V1_GammaFunnelConstants.ROLLER_VOLTS + funnel.rollerVoltageOffset()));
    debugBoard
        .funnel()
        .wheelsOut()
        .whileTrue(
            funnel.setRollerVoltage(
                -(V1_GammaFunnelConstants.ROLLER_VOLTS + funnel.rollerVoltageOffset())));
    debugBoard
        .funnel()
        .incrementFunnelWheelsSpeed()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setRollerVoltageOffset(
                        V1_GammaFunnelConstants.ROLLER_OFFSET_INCREMENT_VOLTS)));
    debugBoard
        .funnel()
        .decrementFunnelWheelsSpeed()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setRollerVoltageOffset(
                        -V1_GammaFunnelConstants.ROLLER_OFFSET_INCREMENT_VOLTS)));
    debugBoard
        .funnel()
        .incrementClosedSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.CLOSED,
                        V1_GammaFunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard
        .funnel()
        .incrementIntakeSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.OPENED,
                        V1_GammaFunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard
        .funnel()
        .incrementStingerOutSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.CLIMB,
                        V1_GammaFunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard
        .funnel()
        .decrementClosedSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.CLOSED,
                        -V1_GammaFunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard
        .funnel()
        .decrementIntakeSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.OPENED,
                        -V1_GammaFunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard
        .funnel()
        .decrementStingerOutSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    funnel.setFunnelPositionOffset(
                        FunnelState.CLIMB,
                        -V1_GammaFunnelConstants.CLAP_DADDY_OFFSET_INCREMENT_RADIANS)));
    debugBoard.funnel().funnelSensorToggle().onTrue(Commands.runOnce(funnel::toggleSensorOverride));

    // Elevator triggers
    debugBoard
        .elevator()
        .stow()
        .onTrue(CompositeCommands.setDynamicReefHeight(ReefHeight.STOW, elevator));
    debugBoard.elevator().raise().onTrue(elevator.setPosition());
    debugBoard.elevator().primeL1().onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L1));
    debugBoard.elevator().primeL2().onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L2));
    debugBoard.elevator().primeL3().onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L3));
    debugBoard.elevator().primeL4().onTrue(CompositeCommands.setStaticReefHeight(ReefHeight.L4));
    debugBoard
        .elevator()
        .decreaseL1Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L1,
                        -V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .decreaseL2Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L2,
                        -V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .decreaseL3Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L3,
                        -V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .decreaseL4Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L4,
                        -V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));

    debugBoard
        .elevator()
        .decreaseStowSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.STOW,
                        -V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseL1Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L1,
                        V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseL2Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L2,
                        V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseL3Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L3,
                        V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseL4Setpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.L4,
                        V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    debugBoard
        .elevator()
        .increaseStowSetpoint()
        .onTrue(
            Commands.runOnce(
                () ->
                    elevator.changeSetpoint(
                        ElevatorPositions.STOW,
                        V1_GammaElevatorConstants.ELEVATOR_HEIGHT_OFFSET_INCREMENT_METERS)));
    // Manipulator triggers
    debugBoard
        .endEffector()
        .wheelsIn()
        .whileTrue(
            manipulator.runManipulator(
                V1_GammaManipulatorConstants.VOLTAGES.RUN_INWARDS_VOLTS().get()));
    debugBoard
        .endEffector()
        .wheelsOut()
        .whileTrue(
            manipulator.runManipulator(
                V1_GammaManipulatorConstants.VOLTAGES.RUN_OUTWARD_VOLTS().get()));
    debugBoard.endEffector().eject().onTrue(ScoreCommands.ejectCoral(elevator, manipulator));
    debugBoard
        .endEffector()
        .incrementSpeed()
        .onTrue(
            Commands.runOnce(
                () ->
                    manipulator.incrementScoreSpeed(
                        V1_GammaManipulatorConstants.VOLTAGES.SCORE_OFFSET_INCREMENT())));
    debugBoard
        .endEffector()
        .decrementSpeed()
        .onTrue(
            Commands.runOnce(
                () ->
                    manipulator.incrementScoreSpeed(
                        -V1_GammaManipulatorConstants.VOLTAGES.SCORE_OFFSET_INCREMENT())));
    debugBoard.endEffector().toggleSensor().onTrue(manipulator.toggleSensorOverride());
    // Climber triggers
    debugBoard.climber().deployLower().onTrue(climber.releaseClimber());
    debugBoard.climber().incrementWintchIn().onTrue(climber.incrementWinchClimber());
    debugBoard.climber().incrementWintchOut().onTrue(climber.decrementWinchClimber());
  }

  private void configureAutos() {
    autoChooser.addOption(
        "Drive FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "3 Piece Left",
        AutonomousCommands.autoALeft(
                drive, elevator, funnel, manipulator, RobotCameras.v1_GammaCams)
            .cmd());
    autoChooser.addOption(
        "3 Piece Right",
        AutonomousCommands.autoARight(
                drive, elevator, funnel, manipulator, RobotCameras.v1_GammaCams)
            .cmd());
    autoChooser.addOption(
        "BlueSide",
        AutonomousCommands.blueSideAuto(
                drive, elevator, funnel, manipulator, RobotCameras.v1_GammaCams)
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
          V1_GammaMechanism3d.getPoses(elevator.getPositionMeters(), funnel.getAngle()));
    }
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
