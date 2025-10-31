package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.Reef.ReefPose;
import frc.robot.FieldConstants.Reef.ReefState;
import frc.robot.subsystems.shared.drive.DriveConstants;
import frc.robot.subsystems.shared.vision.Camera;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ExternalLoggedTracer;
import frc.robot.util.GeometryUtil;
import frc.robot.util.InternalLoggedTracer;
import frc.robot.util.NTPrefixes;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static Rotation2d robotHeading;
  private static Rotation2d headingOffset;
  private static SwerveModulePosition[] modulePositions;

  private static final SwerveDrivePoseEstimator fieldLocalizer;
  private static final SwerveDrivePoseEstimator reefLocalizer;
  private static final SwerveDriveOdometry odometry;

  @Getter private static ReefAlignData reefAlignData;
  @Getter private static OperatorInputData OIData;
  @Getter private static HeadingData headingData;

  @Getter @Setter private static RobotMode mode;
  @Getter @Setter private static boolean hasAlgae;
  @Getter private static ScoreSide scoreSide;

  @Getter @Setter private static boolean isIntakingCoral;
  @Getter @Setter private static boolean isIntakingAlgae;
  @Getter @Setter private static boolean isAutoAligning;
  @Getter @Setter private static boolean autoClapOverride;
  @Getter @Setter private static boolean climberReady;

  private static final TimeInterpolatableBuffer<Pose2d> poseBuffer;

  static {
    switch (Constants.ROBOT) {
      case V0_FUNKY:
      case V0_FUNKY_SIM:
        break;
      case V0_WHIPLASH:
      case V0_WHIPLASH_SIM:
        break;
      case V0_GOMPEIVISION_TEST:
      case V0_GOMPEIVISION_TEST_SIM:
        break;
      case V1_STACKUP:
      case V1_STACKUP_SIM:
        break;
      case V2_REDUNDANCY:
      case V2_REDUNDANCY_SIM:
        break;
      case V3_EPSILON:
      case V3_EPSILON_SIM:
        break;
    }

    OIData = new OperatorInputData(ReefPose.LEFT, ReefState.STOW);
    scoreSide = ScoreSide.RIGHT;

    robotHeading = new Rotation2d();
    headingOffset = new Rotation2d();
    modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = new SwerveModulePosition();
    }

    fieldLocalizer =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_CONFIG.kinematics(),
            new Rotation2d(),
            modulePositions,
            new Pose2d());
    reefLocalizer =
        new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_CONFIG.kinematics(),
            new Rotation2d(),
            modulePositions,
            new Pose2d());
    odometry =
        new SwerveDriveOdometry(
            DriveConstants.DRIVE_CONFIG.kinematics(), new Rotation2d(), modulePositions);

    reefAlignData =
        new ReefAlignData(
            -1, new Pose2d(), new Pose2d(), 0.0, 0.0, false, false, ReefState.ALGAE_INTAKE_BOTTOM);
    headingData = new HeadingData(robotHeading, NetworkTablesJNI.now(), 0.0);

    poseBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
  }

  public RobotState() {}

  public static void periodic(
      Rotation2d robotHeading,
      long latestRobotHeadingTimestamp,
      double robotYawVelocity,
      SwerveModulePosition[] modulePositions,
      Camera[] cameras) {
    ExternalLoggedTracer.reset();
    InternalLoggedTracer.reset();
    RobotState.robotHeading = robotHeading;
    RobotState.modulePositions = modulePositions;
    InternalLoggedTracer.record("Reset Instance Variables", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    fieldLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    reefLocalizer.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);
    odometry.update(robotHeading, modulePositions);

    // Add pose to buffer at timestamp
    poseBuffer.addSample(Timer.getTimestamp(), odometry.getPoseMeters());

    InternalLoggedTracer.record("Update Localizers", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    NetworkTableInstance.getDefault().flush();
    InternalLoggedTracer.record("Flush NT", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    int closestReefTag = getMinDistanceReefTag();
    InternalLoggedTracer.record("Get Minimum Distance To Reef Tag", "RobotState/Periodic");

    // if (RobotMode.disabled()) {
    // resetRobotPose(getRobotPoseField());
    // }

    InternalLoggedTracer.reset();

    // Code will set the robot to the correct position based on the reef tag it is
    // closest to for
    // auto
    // alignment to the reef to score coral

    // --- 1. Get the BASE setpoint (the target location on the field) ---

    Pose2d baseCoralSetpoint =
        OIData.currentReefHeight().equals(ReefState.L1)
            ? Reef.reefMap.get(closestReefTag).getPostSetpoint(ReefPose.CENTER)
            : Reef.reefMap.get(closestReefTag).getPostSetpoint(OIData.currentReefPost());

    Pose2d autoAlignCoralSetpoint = baseCoralSetpoint;

    if (OIData.currentReefHeight().equals(ReefState.L1)) {
      scoreSide = ScoreSide.CENTER;
    } else {

      // --- 2. Automatically determine the closest side of the ROBOT ---
      Pose2d robotPose = getRobotPoseReef();
      Translation2d targetTranslation = baseCoralSetpoint.getTranslation();

      // Define "left" and "right" points on the robot, relative to its center.
      // We'll use the Y-trackwidth from DriveConstants.
      // In WPILib, +Y is "left" and -Y is "right" in robot-local coordinates.
      //
      double robotHalfWidth = DriveConstants.DRIVE_CONFIG.bumperLength() / 2.0;

      // Transform these robot-local points into field-global coordinates
      Translation2d leftSideField =
          robotPose
              .transformBy(new Transform2d(new Translation2d(0, robotHalfWidth), new Rotation2d()))
              .getTranslation();
      Translation2d rightSideField =
          robotPose
              .transformBy(new Transform2d(new Translation2d(0, -robotHalfWidth), new Rotation2d()))
              .getTranslation();

      // Calculate distance from each robot side to the target
      double distToLeft = leftSideField.getDistance(targetTranslation);
      double distToRight = rightSideField.getDistance(targetTranslation);

      // Set the static scoreSide variable based on which side is closer
      // (This variable is now updated every loop)
      if (distToLeft < distToRight) {
        scoreSide = ScoreSide.LEFT;
      } else {
        scoreSide = ScoreSide.RIGHT;
      }

      // --- 3. Now, create the FINAL setpoint using the newly-set scoreSide ---
      if (scoreSide == ScoreSide.RIGHT) {
        autoAlignCoralSetpoint =
            new Pose2d(
                baseCoralSetpoint.getX(),
                baseCoralSetpoint.getY(),
                baseCoralSetpoint.getRotation().rotateBy(new Rotation2d(-Math.PI / 2)));
      } else {
        autoAlignCoralSetpoint =
            new Pose2d(
                baseCoralSetpoint.getX(),
                baseCoralSetpoint.getY(),
                baseCoralSetpoint.getRotation().rotateBy(new Rotation2d(Math.PI / 2)));
      }
    }

    // @Author: Abhiraam Venigalla, Ananth Krishna Gomattam, Atharv Joshi, Chris Xu,
    // Anshu Adiga,
    // Adnan Dembele, Hartej Anand
    // Code will rotate the robot if we need to score in a certain region
    // Rotates the robot relative to the reef to the orientation desired

    Pose2d autoAlignAlgaeSetpoint = Reef.reefMap.get(closestReefTag).getAlgaeSetpoint();
    if (scoreSide == ScoreSide.RIGHT) {
      autoAlignAlgaeSetpoint =
          new Pose2d(
              autoAlignAlgaeSetpoint.getX(),
              autoAlignAlgaeSetpoint.getY(),
              autoAlignAlgaeSetpoint.getRotation().rotateBy(new Rotation2d(-Math.PI / 2)));
    } else if (scoreSide == ScoreSide.LEFT) {
      autoAlignAlgaeSetpoint =
          new Pose2d(
              autoAlignAlgaeSetpoint.getX(),
              autoAlignAlgaeSetpoint.getY(),
              autoAlignAlgaeSetpoint.getRotation().rotateBy(new Rotation2d(Math.PI / 2)));
    }

    double distanceToCoralSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignCoralSetpoint.getTranslation());
    double distanceToAlgaeSetpoint =
        RobotState.getRobotPoseReef()
            .getTranslation()
            .getDistance(autoAlignAlgaeSetpoint.getTranslation());

    boolean atCoralSetpoint =
        Math.abs(distanceToCoralSetpoint)
                <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get()
            && Math.abs(
                    autoAlignCoralSetpoint
                        .getRotation()
                        .minus(getRobotPoseReef().getRotation())
                        .getRadians())
                <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS
                    .omegaPIDConstants()
                    .tolerance()
                    .get();
    boolean atAlgaeSetpoint =
        Math.abs(distanceToAlgaeSetpoint)
                <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS.positionThresholdMeters().get()
            && Math.abs(
                    autoAlignAlgaeSetpoint
                        .getRotation()
                        .minus(getRobotPoseReef().getRotation())
                        .getRadians())
                <= DriveConstants.ALIGN_ROBOT_TO_APRIL_TAG_CONSTANTS
                    .omegaPIDConstants()
                    .tolerance()
                    .get();

    ReefState algaeHeight;
    switch (closestReefTag) {
      case 10:
      case 6:
      case 8:
      case 21:
      case 17:
      case 19:
        algaeHeight = ReefState.ALGAE_INTAKE_BOTTOM;
        break;
      case 9:
      case 11:
      case 7:
      case 22:
      case 20:
      case 18:
        algaeHeight = ReefState.ALGAE_INTAKE_TOP;
        break;
      default:
        algaeHeight = ReefState.ALGAE_INTAKE_BOTTOM;
        break;
    }
    InternalLoggedTracer.record("Generate Setpoints", "RobotState/Periodic");

    InternalLoggedTracer.reset();
    reefAlignData =
        new ReefAlignData(
            closestReefTag,
            autoAlignCoralSetpoint,
            autoAlignAlgaeSetpoint,
            distanceToCoralSetpoint,
            distanceToAlgaeSetpoint,
            atCoralSetpoint,
            atAlgaeSetpoint,
            algaeHeight,
            cameras);

    headingData =
        new HeadingData(
            robotHeading.minus(headingOffset), latestRobotHeadingTimestamp, robotYawVelocity);

    InternalLoggedTracer.record("Generate Records", "RobotState/Periodic");

    Logger.recordOutput(NTPrefixes.ROBOT_STATE + "Has Algae", hasAlgae);

    Logger.recordOutput(NTPrefixes.POSE_DATA + "Field Pose", fieldLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Odometry Pose", odometry.getPoseMeters());
    Logger.recordOutput(NTPrefixes.POSE_DATA + "Heading Offset", headingOffset);

    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Post", OIData.currentReefPost());
    Logger.recordOutput(NTPrefixes.OI_DATA + "Reef Height", OIData.currentReefHeight());

    Logger.recordOutput(NTPrefixes.REEF_DATA + "Reef Pose", reefLocalizer.getEstimatedPosition());
    Logger.recordOutput(NTPrefixes.REEF_DATA + "Closest Reef April Tag", closestReefTag);

    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint", autoAlignCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "Coral Setpoint Error", distanceToCoralSetpoint);
    Logger.recordOutput(NTPrefixes.CORAL_DATA + "At Coral Setpoint", atCoralSetpoint);

    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint", autoAlignAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Setpoint Error", distanceToAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "At Algae Setpoint", atAlgaeSetpoint);
    Logger.recordOutput(NTPrefixes.ALGAE_DATA + "Algae Height", algaeHeight);
    Logger.recordOutput(NTPrefixes.REEF_DATA + "Score Side", scoreSide);
    ExternalLoggedTracer.record("Robot State Total", "RobotState/Periodic");
  }

  public static void addFieldLocalizerVisionMeasurement(VisionObservation observation) {
    if (!GeometryUtil.isZero(observation.pose())) {
      fieldLocalizer.addVisionMeasurement(
          observation.pose(), observation.timestamp(), observation.stddevs());
    }
  }

  public static void addReefLocalizerVisionMeasurement(VisionObservation observation) {
    if (!GeometryUtil.isZero(observation.pose())) {
      reefLocalizer.addVisionMeasurement(
          observation.pose(), observation.timestamp(), observation.stddevs());
    }
  }

  public static Pose2d getRobotPoseField() {
    return fieldLocalizer.getEstimatedPosition();
  }

  public static Pose2d getRobotPoseReef() {
    return reefLocalizer.getEstimatedPosition();
  }

  public static Pose2d getRobotPoseOdometry() {
    return odometry.getPoseMeters();
  }

  private static int getMinDistanceReefTag() {
    int minDistanceTag = 1;
    double minDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 6; i++) {
      double currentDistance =
          RobotState.getRobotPoseReef()
              .getTranslation()
              .getDistance(
                  AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]).getTranslation());
      if (currentDistance < minDistance) {
        minDistance = currentDistance;
        minDistanceTag = i;
      }
    }

    if (AllianceFlipUtil.shouldFlip()) {
      switch (minDistanceTag) {
        case 0:
          minDistanceTag = 7;
          break;
        case 1:
          minDistanceTag = 6;
          break;
        case 2:
          minDistanceTag = 11;
          break;
        case 3:
          minDistanceTag = 10;
          break;
        case 4:
          minDistanceTag = 9;
          break;
        case 5:
          minDistanceTag = 8;
          break;
      }
    } else {
      switch (minDistanceTag) {
        case 0:
          minDistanceTag = 18;
          break;
        case 1:
          minDistanceTag = 19;
          break;
        case 2:
          minDistanceTag = 20;
          break;
        case 3:
          minDistanceTag = 21;
          break;
        case 4:
          minDistanceTag = 22;
          break;
        case 5:
          minDistanceTag = 17;
          break;
      }
    }
    return minDistanceTag;
  }

  public static void resetRobotPose(Pose2d pose) {
    headingOffset = robotHeading.minus(pose.getRotation());
    fieldLocalizer.resetPosition(robotHeading, modulePositions, pose);
    reefLocalizer.resetPosition(robotHeading, modulePositions, pose);
    odometry.resetPosition(robotHeading, modulePositions, pose);
    poseBuffer.clear();
  }

  public static void setReefPost(ReefPose post) {
    ReefState height = OIData.currentReefHeight();
    OIData = new OperatorInputData(post, height);
  }

  public static void toggleReefPost() {
    ReefState height = OIData.currentReefHeight();
    if (OIData.currentReefPost().equals(ReefPose.LEFT)) {
      OIData = new OperatorInputData(ReefPose.RIGHT, height);
    } else {
      OIData = new OperatorInputData(ReefPose.LEFT, height);
    }
  }

  public static void setReefHeight(ReefState height) {
    ReefPose post = OIData.currentReefPost();
    OIData = new OperatorInputData(post, height);
  }

  public static Optional<Pose2d> getBufferedPose(double timestamp) {
    return poseBuffer.getSample(timestamp);
  }

  public static final record ReefAlignData(
      int closestReefTag,
      Pose2d coralSetpoint,
      Pose2d algaeSetpoint,
      double distanceToCoralSetpoint,
      double distanceToAlgaeSetpoint,
      boolean atCoralSetpoint,
      boolean atAlgaeSetpoint,
      ReefState algaeIntakeHeight,
      Camera... cameras) {}

  public static final record OperatorInputData(
      ReefPose currentReefPost, ReefState currentReefHeight) {}

  public static final record HeadingData(
      Rotation2d robotHeading, long latestRobotHeadingTimestamp, double robotYawVelocity) {}

  public static final record VisionObservation(
      Pose2d pose, double timestamp, Matrix<N3, N1> stddevs) {}

  public enum RobotMode {
    DISABLED,
    TELEOP,
    AUTO;

    public static boolean enabled(RobotMode mode) {
      return mode.equals(TELEOP) || mode.equals(AUTO);
    }

    public static boolean disabled(RobotMode mode) {
      return mode.equals(DISABLED);
    }

    public static boolean teleop(RobotMode mode) {
      return mode.equals(TELEOP);
    }

    public static boolean auto(RobotMode mode) {
      return mode.equals(AUTO);
    }

    public static boolean enabled() {
      return enabled(RobotState.getMode());
    }

    public static boolean disabled() {
      return disabled(RobotState.getMode());
    }

    public static boolean teleop() {
      return teleop(RobotState.getMode());
    }

    public static boolean auto() {
      return auto(RobotState.getMode());
    }
  }

  public enum ScoreSide {
    LEFT,
    RIGHT,
    CENTER
  }
}
