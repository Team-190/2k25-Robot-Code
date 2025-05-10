package frc.robot.util;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevator;
import frc.robot.subsystems.v2_Redundancy.superstructure.elevator.V2_RedundancyElevatorConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnel;
import frc.robot.subsystems.v2_Redundancy.superstructure.funnel.V2_RedundancyFunnelConstants;
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

  public static final void updateElevator(V2_RedundancyElevator elevator) {
    LoggedTunableNumber.ifChanged(
        elevator.hashCode(),
        () -> {
          elevator.setGains(
              V2_RedundancyElevatorConstants.GAINS.kP().get(),
              V2_RedundancyElevatorConstants.GAINS.kD().get(),
              V2_RedundancyElevatorConstants.GAINS.kS().get(),
              V2_RedundancyElevatorConstants.GAINS.kV().get(),
              V2_RedundancyElevatorConstants.GAINS.kA().get(),
              V2_RedundancyElevatorConstants.GAINS.kG().get());
          elevator.setConstraints(
              V2_RedundancyElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().get(),
              V2_RedundancyElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get());
        },
        V2_RedundancyElevatorConstants.GAINS.kP(),
        V2_RedundancyElevatorConstants.GAINS.kD(),
        V2_RedundancyElevatorConstants.GAINS.kS(),
        V2_RedundancyElevatorConstants.GAINS.kV(),
        V2_RedundancyElevatorConstants.GAINS.kA(),
        V2_RedundancyElevatorConstants.GAINS.kG(),
        V2_RedundancyElevatorConstants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared(),
        V2_RedundancyElevatorConstants.CONSTRAINTS.cruisingVelocityMetersPerSecond());
  }

  public static final void updateFunnel(V2_RedundancyFunnel funnel) {
    LoggedTunableNumber.ifChanged(
        funnel.hashCode(),
        () -> {
          funnel.updateGains(
              V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kP().get(),
              V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kD().get(),
              V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kS().get(),
              V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kV().get(),
              V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kA().get());
          funnel.updateConstraints(
              V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_ACCELERATION().get(),
              V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_VELOCITY().get());
        },
        V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kP(),
        V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kD(),
        V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kS(),
        V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kV(),
        V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_GAINS.kA(),
        V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_ACCELERATION(),
        V2_RedundancyFunnelConstants.CLAP_DADDY_MOTOR_CONSTRAINTS.MAX_VELOCITY());
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
              V2_RedundancyManipulatorConstants.CONSTRAINTS.maxAccelerationRotationsPerSecondSquared().get(),
              V2_RedundancyManipulatorConstants.CONSTRAINTS.cruisingVelocityRotationsPerSecond().get());
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
