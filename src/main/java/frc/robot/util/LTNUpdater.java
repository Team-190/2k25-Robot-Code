package frc.robot.util;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.Elevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.Funnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.FunnelConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants;

public class LTNUpdater {
  public static final void updateDrive(Drive drive) {
    LoggedTunableNumber.ifChanged(
        drive.hashCode(),
        () -> {
          drive.setPIDGains(
              DriveConstants.GAINS.drive_Kp().get(),
              DriveConstants.GAINS.drive_Kd().get(),
              DriveConstants.GAINS.turn_Kp().get(),
              DriveConstants.GAINS.turn_Kd().get());
          drive.setFFGains(
              DriveConstants.GAINS.drive_Ks().get(), DriveConstants.GAINS.drive_Kv().get());
        },
        DriveConstants.GAINS.drive_Ks(),
        DriveConstants.GAINS.drive_Kv(),
        DriveConstants.GAINS.drive_Kp(),
        DriveConstants.GAINS.drive_Kd(),
        DriveConstants.GAINS.turn_Kp(),
        DriveConstants.GAINS.turn_Kd());

    LoggedTunableNumber.ifChanged(
        drive.hashCode(),
        () -> {
          DriveCommands.setRotationPID(
              DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp().get(),
              DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd().get());
          DriveCommands.setTranslationPID(
              DriveConstants.AUTO_ALIGN_GAINS.translation_Kp().get(),
              DriveConstants.AUTO_ALIGN_GAINS.translation_Kd().get());
        },
        DriveConstants.AUTO_ALIGN_GAINS.rotation_Kp(),
        DriveConstants.AUTO_ALIGN_GAINS.rotation_Kd(),
        DriveConstants.AUTO_ALIGN_GAINS.translation_Kp(),
        DriveConstants.AUTO_ALIGN_GAINS.translation_Kd());
  }

  public static final void updateElevator(Elevator elevator) {
    LoggedTunableNumber.ifChanged(
        elevator.hashCode(),
        () -> {
          elevator.setGains(
              ElevatorConstants.GAINS.kP().get(),
              ElevatorConstants.GAINS.kD().get(),
              ElevatorConstants.GAINS.kS().get(),
              ElevatorConstants.GAINS.kV().get(),
              ElevatorConstants.GAINS.kA().get(),
              ElevatorConstants.GAINS.kG().get());
          elevator.setConstraints(
              ElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().get(),
              ElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get());
        },
        ElevatorConstants.GAINS.kP(),
        ElevatorConstants.GAINS.kD(),
        ElevatorConstants.GAINS.kS(),
        ElevatorConstants.GAINS.kV(),
        ElevatorConstants.GAINS.kA(),
        ElevatorConstants.GAINS.kG(),
        ElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared(),
        ElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond());
  }

  public static final void updateFunnel(Funnel funnel) {
    LoggedTunableNumber.ifChanged(
        funnel.hashCode(),
        () -> {
          funnel.updateGains(
              FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kP().get(),
              FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kD().get(),
              FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kS().get(),
              FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kV().get(),
              FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kA().get());
          funnel.updateConstraints(
              FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get(),
              FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_VELOCITY().get());
        },
        FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kP(),
        FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kD(),
        FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kS(),
        FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kV(),
        FunnelConstants.CLAP_DADDY_MOTOR_GAINS.kA(),
        FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_ACCELERATION(),
        FunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_VELOCITY());
  }

  public static final void updateAlgaeArm(V2_RedundancyManipulator manipulator) {
    LoggedTunableNumber.ifChanged(
        manipulator.hashCode(),
        () -> {
          manipulator.updateArmGains(
              V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kP().get(),
              V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kD().get(),
              V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kS().get(),
              V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kV().get(),
              V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kA().get(),
              V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kG().get(),
              V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kP().get(),
              V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kD().get(),
              V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kS().get(),
              V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kV().get(),
              V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kA().get(),
              V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kG().get());
          manipulator.updateArmConstraints(
              V2_RedundancyManipulatorConstants.CONSTRAINTS
                  .maxAccelerationRotationsPerSecondSquared()
                  .get(),
              V2_RedundancyManipulatorConstants.CONSTRAINTS
                  .cruisingVelocityRotationsPerSecond()
                  .get());
        },
        V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kP(),
        V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kD(),
        V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kS(),
        V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kV(),
        V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kA(),
        V2_RedundancyManipulatorConstants.WITHOUT_ALGAE_GAINS.kG(),
        V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kP(),
        V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kD(),
        V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kS(),
        V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kV(),
        V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kA(),
        V2_RedundancyManipulatorConstants.WITH_ALGAE_GAINS.kG(),
        V2_RedundancyManipulatorConstants.CONSTRAINTS.maxAccelerationRotationsPerSecondSquared(),
        V2_RedundancyManipulatorConstants.CONSTRAINTS.cruisingVelocityRotationsPerSecond());
  }

  public static final void updateIntake(V2_RedundancyIntake intake) {
    LoggedTunableNumber.ifChanged(
        intake.hashCode(),
        () -> {
          intake.updateGains(
              V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kP().get(),
              V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kD().get(),
              V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kS().get(),
              V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kV().get(),
              V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kA().get());
          intake.updateConstraints(
              V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get(),
              V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_VELOCITY().get());
        },
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kP(),
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kD(),
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kS(),
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kV(),
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_GAINS.kA(),
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_ACCELERATION(),
        V2_RedundancyIntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_VELOCITY());
  }
}
