package frc.robot.subsystems.v1_gamma;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevator;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorIO;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorIOSim;
import frc.robot.subsystems.v1_gamma.elevator.V1_GammaElevatorIOTalonFX;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnel;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelIO;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelIOSim;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelIOTalonFX;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulator;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulatorIO;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulatorIOSim;
import frc.robot.subsystems.v1_gamma.manipulator.V1_GammaManipulatorIOTalonFX;
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

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Modes");

  public V1_GammaRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V1_GAMMA:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          vision = new Vision();
          elevator = new V1_GammaElevator(new V1_GammaElevatorIOTalonFX());
          funnel = new V1_GammaFunnel(new V1_GammaFunnelIOTalonFX());
          manipulator = new V1_GammaManipulator(new V1_GammaManipulatorIOTalonFX());
          break;
        case V1_GAMMA_SIM:
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

    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));
  }

  private void configureAutos() {
    autoChooser.addOption(
        "Drive FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
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
          V1_GammaMechanism3d.getPoses(elevator.getPosition(), new Rotation2d()));
    }
  }

  @Override
  public Command getAutonomousCommand() {
    return Commands.sequence(
            elevator.setPosition(ElevatorPositions.L4),
            Commands.waitSeconds(5),
            elevator.setPosition(ElevatorPositions.STOW),
            Commands.waitSeconds(5))
        .repeatedly();
  }
}
