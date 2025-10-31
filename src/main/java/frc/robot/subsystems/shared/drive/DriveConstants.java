package frc.robot.subsystems.shared.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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

  public static final AutoAlignGains AUTO_GAINS;

  public static final double ODOMETRY_FREQUENCY;
  public static final double DRIVER_DEADBAND;
  public static final double OPERATOR_DEADBAND;

  public static final AlignRobotToAprilTagConstants ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS;

  static {
    switch (Constants.ROBOT) {
      case V0_FUNKY:
      case V0_FUNKY_SIM:
        FRONT_LEFT = TunerConstantsV0_Funky.FrontLeft;
        FRONT_RIGHT = TunerConstantsV0_Funky.FrontRight;
        BACK_LEFT = TunerConstantsV0_Funky.BackLeft;
        BACK_RIGHT = TunerConstantsV0_Funky.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsV0_Funky.DrivetrainConstants.CANBusName,
                TunerConstantsV0_Funky.DrivetrainConstants.Pigeon2Id,
                TunerConstantsV0_Funky.kSpeedAt12Volts.in(MetersPerSecond),
                TunerConstantsV0_Funky.kWheelRadius.in(Meters),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT,
                Units.inchesToMeters(34.5),
                Units.inchesToMeters(34.5));

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS", TunerConstantsV0_Funky.driveGains.kS),
                new LoggedTunableNumber("Drive/Drive KV", TunerConstantsV0_Funky.driveGains.kV),
                new LoggedTunableNumber("Drive/Drive KP", TunerConstantsV0_Funky.driveGains.kP),
                new LoggedTunableNumber("Drive/Drive KD", TunerConstantsV0_Funky.driveGains.kD),
                new LoggedTunableNumber("Drive/Turn KP", TunerConstantsV0_Funky.steerGains.kP),
                new LoggedTunableNumber("Drive/Turn KD", TunerConstantsV0_Funky.steerGains.kD));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 40.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 40.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.00));
        AUTO_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Auto Gains/Translation KP", 10.0),
                new LoggedTunableNumber("Drive/Auto Gains/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KD", 0.00));
        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        OPERATOR_DEADBAND = 0.25;
        break;
      case V0_WHIPLASH:
      case V0_WHIPLASH_SIM:
        FRONT_LEFT = TunerConstantsV0_Whiplash.FrontLeft;
        FRONT_RIGHT = TunerConstantsV0_Whiplash.FrontRight;
        BACK_LEFT = TunerConstantsV0_Whiplash.BackLeft;
        BACK_RIGHT = TunerConstantsV0_Whiplash.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsV0_Whiplash.DrivetrainConstants.CANBusName,
                TunerConstantsV0_Whiplash.DrivetrainConstants.Pigeon2Id,
                TunerConstantsV0_Whiplash.kSpeedAt12Volts.in(MetersPerSecond),
                TunerConstantsV0_Whiplash.kWheelRadius.in(Meters),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT,
                Units.inchesToMeters(34.5),
                Units.inchesToMeters(34.5));

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS", TunerConstantsV0_Whiplash.driveGains.kS),
                new LoggedTunableNumber("Drive/Drive KV", TunerConstantsV0_Whiplash.driveGains.kV),
                new LoggedTunableNumber("Drive/Drive KP", TunerConstantsV0_Whiplash.driveGains.kP),
                new LoggedTunableNumber("Drive/Drive KD", TunerConstantsV0_Whiplash.driveGains.kD),
                new LoggedTunableNumber("Drive/Turn KP", TunerConstantsV0_Whiplash.steerGains.kP),
                new LoggedTunableNumber("Drive/Turn KD", TunerConstantsV0_Whiplash.steerGains.kD));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 10.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 10.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));
        AUTO_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Auto Gains/Translation KP", 10.0),
                new LoggedTunableNumber("Drive/Auto Gains/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KD", 0.00));
        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        OPERATOR_DEADBAND = 0.25;
        break;
      case V1_STACKUP:
      case V1_STACKUP_SIM:
      default:
        FRONT_LEFT = TunerConstantsV1_StackUp.FrontLeft;
        FRONT_RIGHT = TunerConstantsV1_StackUp.FrontRight;
        BACK_LEFT = TunerConstantsV1_StackUp.BackLeft;
        BACK_RIGHT = TunerConstantsV1_StackUp.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsV1_StackUp.DrivetrainConstants.CANBusName,
                TunerConstantsV1_StackUp.DrivetrainConstants.Pigeon2Id,
                TunerConstantsV1_StackUp.kSpeedAt12Volts.in(MetersPerSecond),
                TunerConstantsV1_StackUp.kWheelRadius.in(Meters),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT,
                Units.inchesToMeters(34.5),
                Units.inchesToMeters(34.5));

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS", TunerConstantsV1_StackUp.driveGains.kS),
                new LoggedTunableNumber("Drive/Drive KV", TunerConstantsV1_StackUp.driveGains.kV),
                new LoggedTunableNumber("Drive/Drive KP", TunerConstantsV1_StackUp.driveGains.kP),
                new LoggedTunableNumber("Drive/Drive KD", TunerConstantsV1_StackUp.driveGains.kD),
                new LoggedTunableNumber("Drive/Turn KP", TunerConstantsV1_StackUp.steerGains.kP),
                new LoggedTunableNumber("Drive/Turn KD", TunerConstantsV1_StackUp.steerGains.kD));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 4.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));

        AUTO_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Auto Gains/Translation KP", 10.0),
                new LoggedTunableNumber("Drive/Auto Gains/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KD", 0.00));

        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        OPERATOR_DEADBAND = 0.25;
        break;
      case V2_REDUNDANCY:
      case V2_REDUNDANCY_SIM:
        FRONT_LEFT = TunerConstantsV2_Redundancy.FrontLeft;
        FRONT_RIGHT = TunerConstantsV2_Redundancy.FrontRight;
        BACK_LEFT = TunerConstantsV2_Redundancy.BackLeft;
        BACK_RIGHT = TunerConstantsV2_Redundancy.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsV2_Redundancy.DrivetrainConstants.CANBusName,
                TunerConstantsV2_Redundancy.DrivetrainConstants.Pigeon2Id,
                TunerConstantsV2_Redundancy.kSpeedAt12Volts.in(MetersPerSecond),
                TunerConstantsV2_Redundancy.kWheelRadius.in(Meters),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT,
                Units.inchesToMeters(34.5),
                Units.inchesToMeters(34.5));

        GAINS =
            new Gains(
                new LoggedTunableNumber(
                    "Drive/Drive KS", TunerConstantsV2_Redundancy.driveGains.kS),
                new LoggedTunableNumber(
                    "Drive/Drive KV", TunerConstantsV2_Redundancy.driveGains.kV),
                new LoggedTunableNumber(
                    "Drive/Drive KP", TunerConstantsV2_Redundancy.driveGains.kP),
                new LoggedTunableNumber(
                    "Drive/Drive KD", TunerConstantsV2_Redundancy.driveGains.kD),
                new LoggedTunableNumber("Drive/Turn KP", TunerConstantsV2_Redundancy.steerGains.kP),
                new LoggedTunableNumber(
                    "Drive/Turn KD", TunerConstantsV2_Redundancy.steerGains.kD));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 4.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));
        AUTO_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Auto Gains/Translation KP", 10.0),
                new LoggedTunableNumber("Drive/Auto Gains/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KD", 0.00));
        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        OPERATOR_DEADBAND = 0.25;
        break;
      case V3_EPSILON:
      case V3_EPSILON_SIM:
        FRONT_LEFT = TunerConstantsV3_Epsilon.FrontLeft;
        FRONT_RIGHT = TunerConstantsV3_Epsilon.FrontRight;
        BACK_LEFT = TunerConstantsV3_Epsilon.BackLeft;
        BACK_RIGHT = TunerConstantsV3_Epsilon.BackRight;

        DRIVE_CONFIG =
            new DriveConfig(
                TunerConstantsV3_Epsilon.DrivetrainConstants.CANBusName,
                TunerConstantsV3_Epsilon.DrivetrainConstants.Pigeon2Id,
                TunerConstantsV3_Epsilon.kSpeedAt12Volts.in(MetersPerSecond),
                TunerConstantsV3_Epsilon.kWheelRadius.in(Meters),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX60Foc(1),
                FRONT_LEFT,
                FRONT_RIGHT,
                BACK_LEFT,
                BACK_RIGHT,
                Units.inchesToMeters(33.5),
                Units.inchesToMeters(33.5));

        GAINS =
            new Gains(
                new LoggedTunableNumber("Drive/Drive KS", TunerConstantsV3_Epsilon.driveGains.kS),
                new LoggedTunableNumber("Drive/Drive KV", TunerConstantsV3_Epsilon.driveGains.kV),
                new LoggedTunableNumber("Drive/Drive KP", TunerConstantsV3_Epsilon.driveGains.kP),
                new LoggedTunableNumber("Drive/Drive KD", TunerConstantsV3_Epsilon.driveGains.kD),
                new LoggedTunableNumber("Drive/Turn KP", TunerConstantsV3_Epsilon.steerGains.kP),
                new LoggedTunableNumber("Drive/Turn KD", TunerConstantsV3_Epsilon.steerGains.kD));
        AUTO_ALIGN_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Translation KP", 4.0),
                new LoggedTunableNumber("Drive/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Rotation KD", 0.05));
        AUTO_GAINS =
            new AutoAlignGains(
                new LoggedTunableNumber("Drive/Auto Gains/Translation KP", 10.0),
                new LoggedTunableNumber("Drive/Auto Gains/Translation KD", 0.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KP", 5.0),
                new LoggedTunableNumber("Drive/Auto Gains/Rotation KD", 0.00));
        ODOMETRY_FREQUENCY = 250.0;
        DRIVER_DEADBAND = 0.025;
        OPERATOR_DEADBAND = 0.25;
        break;
    }
    ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS =
        new AlignRobotToAprilTagConstants(
            new PIDControllerConstants(
                new LoggedTunableNumber("Drive/Align Robot To April Tag/X Constants/kP", 3),
                new LoggedTunableNumber("Drive/Align Robot To April Tag/X Constants/kD", 0.15),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/X Constants/tolerance", 0.03),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/X Constants/maxVelocity", 2.5)),
            new PIDControllerConstants(
                new LoggedTunableNumber("Drive/Align Robot To April Tag/Y Constants/kP", 3),
                new LoggedTunableNumber("Drive/Align Robot To April Tag/Y Constants/kD", 0.15),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Y Constants/tolerance", 0.03),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Y Constants/maxVelocity", 2.5)),
            new PIDControllerConstants(
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Omega Constants/kP", 2 * Math.PI),
                new LoggedTunableNumber("Drive/Align Robot To April Tag/Omega Constants/kD", 0.05),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Omega Constants/tolerance",
                    Units.degreesToRadians(1)),
                new LoggedTunableNumber(
                    "Drive/Align Robot To April Tag/Omega Constants/maxVelocity", Math.PI)),
            new LoggedTunableNumber(
                "Drive/Align Robot To April Tag/positionThresholdDegrees", 0.02));
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
          backRight,
      double bumperWidth,
      double bumperLength) {
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

  public record PIDControllerConstants(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber tolerance,
      LoggedTunableNumber maxVelocity) {}

  public static record AlignRobotToAprilTagConstants(
      PIDControllerConstants xPIDConstants,
      PIDControllerConstants yPIDConstants,
      PIDControllerConstants omegaPIDConstants,
      LoggedTunableNumber positionThresholdMeters) {}
}
