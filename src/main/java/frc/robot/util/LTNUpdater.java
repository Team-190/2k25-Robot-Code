package frc.robot.util;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.elevator.Elevator;
import frc.robot.subsystems.shared.elevator.ElevatorConstants;
import frc.robot.subsystems.shared.funnel.Funnel;
import frc.robot.subsystems.shared.funnel.FunnelConstants;

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
}
