package frc.robot.util;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.Elevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.Funnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.FunnelConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.Intake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.IntakeConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.Manipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.ManipulatorConstants;

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

  public static final void updateAlgaeArm(Manipulator manipulator) {
    LoggedTunableNumber.ifChanged(
        manipulator.hashCode(),
        () -> {
          manipulator.updateArmGains(
              ManipulatorConstants.WITHOUT_ALGAE_GAINS.kP().get(),
              ManipulatorConstants.WITHOUT_ALGAE_GAINS.kD().get(),
              ManipulatorConstants.WITHOUT_ALGAE_GAINS.kS().get(),
              ManipulatorConstants.WITHOUT_ALGAE_GAINS.kV().get(),
              ManipulatorConstants.WITHOUT_ALGAE_GAINS.kA().get(),
              ManipulatorConstants.WITHOUT_ALGAE_GAINS.kG().get(),
              ManipulatorConstants.WITH_ALGAE_GAINS.kP().get(),
              ManipulatorConstants.WITH_ALGAE_GAINS.kD().get(),
              ManipulatorConstants.WITH_ALGAE_GAINS.kS().get(),
              ManipulatorConstants.WITH_ALGAE_GAINS.kV().get(),
              ManipulatorConstants.WITH_ALGAE_GAINS.kA().get(),
              ManipulatorConstants.WITH_ALGAE_GAINS.kG().get());
          manipulator.updateArmConstraints(
              ManipulatorConstants.CONSTRAINTS.maxAccelerationRotationsPerSecondSquared().get(),
              ManipulatorConstants.CONSTRAINTS.cruisingVelocityRotationsPerSecond().get());
        },
        ManipulatorConstants.WITHOUT_ALGAE_GAINS.kP(),
        ManipulatorConstants.WITHOUT_ALGAE_GAINS.kD(),
        ManipulatorConstants.WITHOUT_ALGAE_GAINS.kS(),
        ManipulatorConstants.WITHOUT_ALGAE_GAINS.kV(),
        ManipulatorConstants.WITHOUT_ALGAE_GAINS.kA(),
        ManipulatorConstants.WITHOUT_ALGAE_GAINS.kG(),
        ManipulatorConstants.WITH_ALGAE_GAINS.kP(),
        ManipulatorConstants.WITH_ALGAE_GAINS.kD(),
        ManipulatorConstants.WITH_ALGAE_GAINS.kS(),
        ManipulatorConstants.WITH_ALGAE_GAINS.kV(),
        ManipulatorConstants.WITH_ALGAE_GAINS.kA(),
        ManipulatorConstants.WITH_ALGAE_GAINS.kG(),
        ManipulatorConstants.CONSTRAINTS.maxAccelerationRotationsPerSecondSquared(),
        ManipulatorConstants.CONSTRAINTS.cruisingVelocityRotationsPerSecond());
  }

  public static final void updateIntake(Intake intake) {
    LoggedTunableNumber.ifChanged(
        intake.hashCode(),
        () -> {
          intake.updateGains(
              IntakeConstants.EXTENSION_MOTOR_GAINS.kP().get(),
              IntakeConstants.EXTENSION_MOTOR_GAINS.kD().get(),
              IntakeConstants.EXTENSION_MOTOR_GAINS.kS().get(),
              IntakeConstants.EXTENSION_MOTOR_GAINS.kV().get(),
              IntakeConstants.EXTENSION_MOTOR_GAINS.kA().get());
          intake.updateConstraints(
              IntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get(),
              IntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_VELOCITY().get());
        },
        IntakeConstants.EXTENSION_MOTOR_GAINS.kP(),
        IntakeConstants.EXTENSION_MOTOR_GAINS.kD(),
        IntakeConstants.EXTENSION_MOTOR_GAINS.kS(),
        IntakeConstants.EXTENSION_MOTOR_GAINS.kV(),
        IntakeConstants.EXTENSION_MOTOR_GAINS.kA(),
        IntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_ACCELERATION(),
        IntakeConstants.EXTENSION_MOTOR_CONSTRAINTS.MAX_VELOCITY());
  }
}
