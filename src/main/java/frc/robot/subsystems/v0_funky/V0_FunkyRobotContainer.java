package frc.robot.subsystems.v0_funky;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants.Reef.ReefPost;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.vision.CameraConstants;
import frc.robot.subsystems.shared.vision.Vision;
import frc.robot.subsystems.v0_funky.kitbot_roller.V0_FunkyRoller;
import frc.robot.subsystems.v0_funky.kitbot_roller.V0_FunkyRollerIO;
import frc.robot.subsystems.v0_funky.kitbot_roller.V0_FunkyRollerIOTalonFX;
import frc.robot.util.LTNUpdater;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class V0_FunkyRobotContainer implements RobotContainer {

  // Subsystems
  private Drive drive;
  private Vision vision;

  private V0_FunkyRoller roller;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);

  // Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Modes");

  public V0_FunkyRobotContainer() {

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case V0_FUNKY:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(DriveConstants.BACK_RIGHT));
          vision = new Vision(CameraConstants.RobotCameras.v0_FunkyCams);
          roller = new V0_FunkyRoller(new V0_FunkyRollerIOTalonFX());
          break;
        case V0_FUNKY_SIM:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(DriveConstants.FRONT_LEFT),
                  new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                  new ModuleIOSim(DriveConstants.BACK_LEFT),
                  new ModuleIOSim(DriveConstants.BACK_RIGHT));
          vision = new Vision();
          roller = new V0_FunkyRoller(new V0_FunkyRollerIO() {});
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
    if (roller == null) {
      roller = new V0_FunkyRoller(new V0_FunkyRollerIO() {});
    }

    configureButtonBindings();
    configureAutos();
  }

  public void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(), ()->false));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));

    roller.setDefaultCommand(
        roller.runRoller(() -> driver.getLeftTriggerAxis(), () -> driver.getRightTriggerAxis()));

    driver.a().whileTrue(DriveCommands.alignRobotToAprilTag(drive, vision.getCameras()));

    driver.povLeft().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPost.LEFT)));
    driver.povRight().onTrue(Commands.runOnce(() -> RobotState.setReefPost(ReefPost.RIGHT)));
  }

  public void configureAutos() {
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
  }

  @Override
  public Command getAutonomousCommand() {
    return AutonomousCommands.autoBLeft(drive).cmd();
  }
}
