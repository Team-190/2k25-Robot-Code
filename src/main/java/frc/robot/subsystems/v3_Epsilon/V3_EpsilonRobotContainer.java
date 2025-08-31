package frc.robot.subsystems.v3_Epsilon;

import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
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

  // Controller

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
          superstructure = new V3_EpsilonSuperstructure(elevator, intake, manipulator);
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
          superstructure = new V3_EpsilonSuperstructure(elevator, intake, manipulator);
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
      if (superstructure == null) {
        superstructure = new V3_EpsilonSuperstructure(elevator, intake, manipulator);
      }
    }
  }

  private void configureButtonBindings() {}

  private void configureAutos() {}

  @Override
  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRawGyroRotation(),
        NetworkTablesJNI.now(),
        drive.getYawVelocity(),
        drive.getModulePositions(),
        null);

    LTNUpdater.updateDrive(drive);
    LTNUpdater.updateElevator(elevator);

    Logger.recordOutput(
        "Component Poses",
        V3_EpsilonMechanism3d.getPoses(
            elevator.getPositionMeters(), intake.getPivotAngle(), manipulator.getArmAngle()));
  }

  @Override
  public Command getAutonomousCommand() {
    return superstructure.runGoal(V3_EpsilonSuperstructureStates.BARGE_SCORE);
  }
}
