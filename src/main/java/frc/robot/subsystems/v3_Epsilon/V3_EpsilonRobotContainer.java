package frc.robot.subsystems.v3_Epsilon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.CompositeCommands.V3_EpsilonCompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.shared.elevator.ElevatorIO;
import frc.robot.subsystems.shared.elevator.ElevatorIOSim;
import frc.robot.subsystems.shared.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.shared.vision.VisionConstants.RobotCameras;
import frc.robot.subsystems.v3_Epsilon.climber.V3_EpsilonClimber;
import frc.robot.subsystems.v3_Epsilon.climber.V3_EpsilonClimberIO;
import frc.robot.subsystems.v3_Epsilon.climber.V3_EpsilonClimberIOSim;
import frc.robot.subsystems.v3_Epsilon.climber.V3_EpsilonClimberIOTalonFX;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructure;
import frc.robot.subsystems.v3_Epsilon.superstructure.V3_EpsilonSuperstructureStates;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIO;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIOSim;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeIOTalonFX;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorIO;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorIOSim;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorIOTalonFX;
import frc.robot.util.LTNUpdater;
import org.littletonrobotics.junction.Logger;

public class V3_EpsilonRobotContainer implements RobotContainer {
  // Subsystems
  private Drive drive;
  private ElevatorFSM elevator;
  private V3_EpsilonIntake intake;
  private V3_EpsilonManipulator manipulator;
  private V3_EpsilonSuperstructure superstructure;
  private V3_EpsilonClimber climber;
  private Vision vision;

  // Controller
  CommandXboxController driver = new CommandXboxController(0);

  // Auto chooser

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
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
                  RobotCameras.V2_REDUNDANCY_CAMS);
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
          vision =
              new Vision(() -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
          break;
        default:
          break;
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
      if (superstructure == null) {
        superstructure = new V3_EpsilonSuperstructure(elevator, intake, manipulator);
      }
    }
  }


  /**
   * Configure the button bindings for the robot. This method is called in
   * the constructor and is responsible for setting up the default commands
   * for each button on the controllers.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false,
            driver.back(),
            driver.povRight()));
    driver
        .rightTrigger()
        .whileTrue(
            V3_EpsilonCompositeCommands.intakeCoralDriverSequence(
                superstructure, intake, manipulator))
        .whileFalse(superstructure.runGoal(V3_EpsilonSuperstructureStates.HANDOFF));
  }

  private void configureAutos() {}


/**
 * Periodic function for the robot. This function is called every 20ms,
 * and is responsible for updating the robot's state and logging relevant
 * data.
 */
  @Override
  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getModulePositions(),
        vision.getCameras());

    LTNUpdater.updateDrive(drive);
    LTNUpdater.updateElevator(elevator);

    Logger.recordOutput(
        "Component Poses",
        V3_EpsilonMechanism3d.getPoses(
            elevator.getPositionMeters(), intake.getPivotAngle(), manipulator.getArmAngle()));
  }

  /**
   * Returns the autonomous command for the robot. This command will be scheduled
   * for the entire autonomous period. 
   *
   * @return the autonomous command for the robot
   */
/*******  a3f2747b-04ef-49d3-bd1f-182e56d47707  *******/
  public Command getAutonomousCommand() {
    // return superstructure.allTransition();
    return CompositeCommands.V3_EpsilonCompositeCommands.climb(
        superstructure, drive, climber, intake, manipulator);
  }
}
