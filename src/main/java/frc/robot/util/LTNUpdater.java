package frc.robot.util;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.v1_gamma.funnel.Funnel;
import frc.robot.subsystems.v1_gamma.funnel.FunnelConstants;

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

  public static final void updateFunnel(Funnel funnel) {
    LoggedTunableNumber.ifChanged(
        funnel.hashCode(),
        () -> {
          funnel.updateGains(
              FunnelConstants.SERIALIZER_MOTOR_GAINS.kP().get(),
              FunnelConstants.SERIALIZER_MOTOR_GAINS.kD().get(),
              FunnelConstants.SERIALIZER_MOTOR_GAINS.kS().get(),
              FunnelConstants.SERIALIZER_MOTOR_GAINS.kV().get(),
              FunnelConstants.SERIALIZER_MOTOR_GAINS.kA().get());
          funnel.updateConstraints(
              FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get(),
              FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get());
          funnel.updateThresholds(
              FunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS().get(),
              FunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS().get());
        },
        FunnelConstants.SERIALIZER_MOTOR_GAINS.kP(),
        FunnelConstants.SERIALIZER_MOTOR_GAINS.kD(),
        FunnelConstants.SERIALIZER_MOTOR_GAINS.kS(),
        FunnelConstants.SERIALIZER_MOTOR_GAINS.kV(),
        FunnelConstants.SERIALIZER_MOTOR_GAINS.kA(),
        FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_ACCELERATION(),
        FunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_VELOCITY(),
        FunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS(),
        FunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS());
  }
}
