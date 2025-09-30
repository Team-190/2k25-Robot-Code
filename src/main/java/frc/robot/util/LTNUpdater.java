package frc.robot.util;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.shared.drive.Drive;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorCSB;
import frc.robot.subsystems.shared.elevator.Elevator.ElevatorFSM;
import frc.robot.subsystems.shared.elevator.ElevatorConstants;
import frc.robot.subsystems.shared.funnel.Funnel.FunnelCSB;
import frc.robot.subsystems.shared.funnel.Funnel.FunnelFSM;
import frc.robot.subsystems.shared.funnel.FunnelConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntake;
import frc.robot.subsystems.v2_Redundancy.superstructure.intake.V2_RedundancyIntakeConstants;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulator;
import frc.robot.subsystems.v2_Redundancy.superstructure.manipulator.V2_RedundancyManipulatorConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntake;
import frc.robot.subsystems.v3_Epsilon.superstructure.intake.V3_EpsilonIntakeConstants;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulator;
import frc.robot.subsystems.v3_Epsilon.superstructure.manipulator.V3_EpsilonManipulatorConstants;

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

    public static final void updateElevator(ElevatorCSB elevator) {
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

    public static final void updateElevator(ElevatorFSM elevator) {
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

    public static final void updateFunnel(FunnelCSB funnel) {
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

    public static final void updateFunnel(FunnelFSM funnel) {
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
                                    .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED()
                                    .get(),
                            V2_RedundancyManipulatorConstants.CONSTRAINTS
                                    .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED()
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
                V2_RedundancyManipulatorConstants.CONSTRAINTS
                        .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED(),
                V2_RedundancyManipulatorConstants.CONSTRAINTS
                        .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED());
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
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kP(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kD(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kS(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kV(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kA(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kG(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kP(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kD(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kS(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kV(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kA(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kG(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kP(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kD(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kS(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kV(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kA(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kG(),
                V3_EpsilonManipulatorConstants.CONSTRAINTS
                        .maxAccelerationRotationsPerSecondSquared(),
                V3_EpsilonManipulatorConstants.CONSTRAINTS
                        .cruisingVelocityRotationsPerSecond(),
                V3_EpsilonManipulatorConstants.CONSTRAINTS
                        .goalToleranceRadians());
    }

    public static final void updateIntake(V3_EpsilonIntake intake) {
        LoggedTunableNumber.ifChanged(
                intake.hashCode(),
                () -> {
                    intake.updateIntakeGains(
                            V3_EpsilonIntakeConstants.PIVOT_GAINS.kP().get(),
                            V3_EpsilonIntakeConstants.PIVOT_GAINS.kD().get(),
                            V3_EpsilonIntakeConstants.PIVOT_GAINS.kS().get(),
                            V3_EpsilonIntakeConstants.PIVOT_GAINS.kV().get(),
                            V3_EpsilonIntakeConstants.PIVOT_GAINS.kA().get(),
                            V3_EpsilonIntakeConstants.PIVOT_GAINS.kG().get());
                    intake.updateIntakeConstraints(
                            V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED()
                                    .get(),
                            V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND().get());
                },
                V3_EpsilonIntakeConstants.PIVOT_GAINS.kP(),
                V3_EpsilonIntakeConstants.PIVOT_GAINS.kD(),
                V3_EpsilonIntakeConstants.PIVOT_GAINS.kS(),
                V3_EpsilonIntakeConstants.PIVOT_GAINS.kV(),
                V3_EpsilonIntakeConstants.PIVOT_GAINS.kA(),
                V3_EpsilonIntakeConstants.PIVOT_GAINS.kG(),
                V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED(),
                V3_EpsilonIntakeConstants.PIVOT_CONSTRAINTS.CRUISING_VELOCITY_RADIANS_PER_SECOND());
    }

    public static final void updateManipulatorArm(V3_EpsilonManipulator manipulator) {
        LoggedTunableNumber.ifChanged(
                manipulator.hashCode(),
                () -> {
                    manipulator.updateArmGains(
                            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kP().get(),
                            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kD().get(),
                            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kS().get(),
                            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kV().get(),
                            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kA().get(),
                            V3_EpsilonManipulatorConstants.EMPTY_GAINS.kG().get(),
                            V3_EpsilonManipulatorConstants.CORAL_GAINS.kP().get(),
                            V3_EpsilonManipulatorConstants.CORAL_GAINS.kD().get(),
                            V3_EpsilonManipulatorConstants.CORAL_GAINS.kS().get(),
                            V3_EpsilonManipulatorConstants.CORAL_GAINS.kV().get(),
                            V3_EpsilonManipulatorConstants.CORAL_GAINS.kA().get(),
                            V3_EpsilonManipulatorConstants.CORAL_GAINS.kG().get(),
                            V3_EpsilonManipulatorConstants.ALGAE_GAINS.kP().get(),
                            V3_EpsilonManipulatorConstants.ALGAE_GAINS.kD().get(),
                            V3_EpsilonManipulatorConstants.ALGAE_GAINS.kS().get(),
                            V3_EpsilonManipulatorConstants.ALGAE_GAINS.kV().get(),
                            V3_EpsilonManipulatorConstants.ALGAE_GAINS.kA().get(),
                            V3_EpsilonManipulatorConstants.ALGAE_GAINS.kG().get());
                    manipulator.updateArmConstraints(
                            V3_EpsilonManipulatorConstants.CONSTRAINTS
                                    .maxAccelerationRotationsPerSecondSquared()
                                    .get(),
                            V3_EpsilonManipulatorConstants.CONSTRAINTS
                                    .cruisingVelocityRotationsPerSecond()
                                    .get());
                },
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kP(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kD(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kS(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kV(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kA(),
                V3_EpsilonManipulatorConstants.ALGAE_GAINS.kG(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kP(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kD(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kS(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kV(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kA(),
                V3_EpsilonManipulatorConstants.EMPTY_GAINS.kG(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kP(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kD(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kS(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kV(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kA(),
                V3_EpsilonManipulatorConstants.CORAL_GAINS.kG(),
                V3_EpsilonManipulatorConstants.CONSTRAINTS
                        .maxAccelerationRotationsPerSecondSquared(),
                V3_EpsilonManipulatorConstants.CONSTRAINTS
                        .cruisingVelocityRotationsPerSecond(),
                V3_EpsilonManipulatorConstants.CONSTRAINTS
                        .goalToleranceRadians());
    }

}
