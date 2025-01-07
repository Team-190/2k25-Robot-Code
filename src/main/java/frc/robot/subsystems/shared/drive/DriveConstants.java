package frc.robot.subsystems.shared.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class DriveConstants {
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT;

  public static final DriveConfig DRIVE_CONFIG;

  public static final Gains GAINS;
  public static final AutoAlignGains AUTO_ALIGN_GAINS;

  public static final double ODOMETRY_FREQUENCY;
  public static final double DRIVER_DEADBAND;

  static {
    switch (Constants.ROBOT) {
      case V1_GAMMA:
      case V1_GAMMA_SIM:
      default:
        FRONT_LEFT = TunerConstantsV1_Gamma.FrontLeft;
        FRONT_RIGHT = TunerConstantsV1_Gamma.FrontRight;
        BACK_LEFT = TunerConstantsV1_Gamma.BackLeft;
        BACK_RIGHT = TunerConstantsV1_Gamma.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsV1_Gamma.DrivetrainConstants.CANBusName,
                TunerConstantsV1_Gamma.DrivetrainConstants.Pigeon2Id,
                3.0,
                2.0,
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT);

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS", 0.0),
                new LoggedTunableNumber("Drive/Drive KV", 0.0),
                new LoggedTunableNumber("Drive/Drive KP", 0.0),
                new LoggedTunableNumber("Drive/Drive KD", 0.0),
                new LoggedTunableNumber("Drive/Turn KP", 0.0),
                new LoggedTunableNumber("Drive/Turn KD", 0.0));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 4.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));

        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        break;
      case V2_DELTA:
      case V2_DELTA_SIM:
        FRONT_LEFT = TunerConstantsV2_Delta.FrontLeft;
        FRONT_RIGHT = TunerConstantsV2_Delta.FrontRight;
        BACK_LEFT = TunerConstantsV2_Delta.BackLeft;
        BACK_RIGHT = TunerConstantsV2_Delta.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsV2_Delta.DrivetrainConstants.CANBusName,
                TunerConstantsV2_Delta.DrivetrainConstants.Pigeon2Id,
                3.0,
                2.0,
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT);

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS"),
                new LoggedTunableNumber("Drive/Drive KV"),
                new LoggedTunableNumber("Drive/Drive KP"),
                new LoggedTunableNumber("Drive/Drive KD"),
                new LoggedTunableNumber("Drive/Turn KP"),
                new LoggedTunableNumber("Drive/Turn KD"));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 4.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));
        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        break;
      case FUNKY:
      case FUNKY_SIM:
        FRONT_LEFT = TunerConstantsFunky.FrontLeft;
        FRONT_RIGHT = TunerConstantsFunky.FrontRight;
        BACK_LEFT = TunerConstantsFunky.BackLeft;
        BACK_RIGHT = TunerConstantsFunky.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsFunky.DrivetrainConstants.CANBusName,
                TunerConstantsFunky.DrivetrainConstants.Pigeon2Id,
                3.0,
                2.0,
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT);

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS"),
                new LoggedTunableNumber("Drive/Drive KV"),
                new LoggedTunableNumber("Drive/Drive KP"),
                new LoggedTunableNumber("Drive/Drive KD"),
                new LoggedTunableNumber("Drive/Turn KP"),
                new LoggedTunableNumber("Drive/Turn KD"));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 4.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));
        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        break;
    }
  }

  public record DriveConfig(
      String canBus,
      int pigeon2Id,
      double maxLinearVelocityMetersPerSecond,
      double wheelRadiusMeters,
      DCMotor driveModel,
      DCMotor turnModel,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frontLeft,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frontRight,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          backLeft,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          backRight) {
    public double driveBaseRadius() {
      return Math.hypot(
          (Math.abs(frontLeft.LocationX) + Math.abs(frontRight.LocationX)) / 2.0,
          (Math.abs(frontLeft.LocationY) + Math.abs(backLeft.LocationY)) / 2.0);
    }

    public double maxAngularVelocity() {
      return maxLinearVelocityMetersPerSecond / driveBaseRadius();
    }

    public Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(frontLeft.LocationX, frontLeft.LocationY),
        new Translation2d(frontRight.LocationX, frontRight.LocationY),
        new Translation2d(backLeft.LocationX, backLeft.LocationY),
        new Translation2d(backRight.LocationX, backRight.LocationY)
      };
    }

    public SwerveDriveKinematics kinematics() {
      return new SwerveDriveKinematics(getModuleTranslations());
    }
  }

  public record Gains(
      LoggedTunableNumber drive_Ks,
      LoggedTunableNumber drive_Kv,
      LoggedTunableNumber drive_Kp,
      LoggedTunableNumber drive_Kd,
      LoggedTunableNumber turn_Kp,
      LoggedTunableNumber turn_Kd) {}

  public record AutoAlignGains(
      LoggedTunableNumber translation_Kp,
      LoggedTunableNumber translation_Kd,
      LoggedTunableNumber rotation_Kp,
      LoggedTunableNumber rotation_Kd) {}
}
