package frc.robot.subsystems.v0_Funky;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.RobotContainer;
import frc.robot.RobotStateLL;
import frc.robot.commands.CompositeCommands.SharedCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.drive.GyroIO;
import frc.robot.subsystems.shared.drive.GyroIOPigeon2;
import frc.robot.subsystems.shared.drive.ModuleIO;
import frc.robot.subsystems.shared.drive.ModuleIOSim;
import frc.robot.subsystems.shared.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shared.visionlimelight.CameraConstants;
import frc.robot.subsystems.shared.visionlimelight.Vision;
import frc.robot.subsystems.v0_Funky.kitbot_roller.V0_FunkyRoller;
import frc.robot.subsystems.v0_Funky.kitbot_roller.V0_FunkyRollerIO;
import frc.robot.subsystems.v0_Funky.kitbot_roller.V0_FunkyRollerIOTalonFX;
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
                  new ModuleIOTalonFX(0, DriveConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(1, DriveConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(2, DriveConstants.BACK_LEFT),
                  new ModuleIOTalonFX(3, DriveConstants.BACK_RIGHT));
          vision = new Vision(CameraConstants.RobotCameras.V0_FUNKY_CAMS);
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
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> driver.b().getAsBoolean()));
    driver.y().onTrue(SharedCommands.resetHeading(drive));

    roller.setDefaultCommand(
        roller.runRoller(() -> driver.getLeftTriggerAxis(), () -> driver.getRightTriggerAxis()));

    driver.a().whileTrue(DriveCommands.autoAlignReefCoral(drive, vision.getCameras()));

    driver.povLeft().onTrue(Commands.runOnce(() -> RobotStateLL.setReefPost(ReefPose.LEFT)));
    driver.povRight().onTrue(Commands.runOnce(() -> RobotStateLL.setReefPost(ReefPose.RIGHT)));
  }

  public void configureAutos() {
    autoChooser.addOption(
        "Drive FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
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
  }

  @Override
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
