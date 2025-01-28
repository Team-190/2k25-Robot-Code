package frc.robot.util;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnel;
import frc.robot.subsystems.v1_gamma.funnel.V1_GammaFunnelConstants;

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

  public static final void updateFunnel(V1_GammaFunnel funnel) {
    LoggedTunableNumber.ifChanged(
        funnel.hashCode(),
        () -> {
          funnel.updateGains(
              V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kP().get(),
              V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kD().get(),
              V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kS().get(),
              V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kV().get(),
              V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kA().get());
          funnel.updateConstraints(
              V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get(),
              V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_VELOCITY().get());
          funnel.updateThresholds(
              V1_GammaFunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS().get(),
              V1_GammaFunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS().get());
        },
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kP(),
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kD(),
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kS(),
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kV(),
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_GAINS.kA(),
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_ACCELERATION(),
        V1_GammaFunnelConstants.SERIALIZER_MOTOR_CONSTRAINTS.MAX_VELOCITY(),
        V1_GammaFunnelConstants.ANGLE_THRESHOLDS.MAX_ANGLE_RADIANS(),
        V1_GammaFunnelConstants.ANGLE_THRESHOLDS.MIN_ANGLE_RADIANS());
  }
}
