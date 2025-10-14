package frc.robot.subsystems.v3_Epsilon.superstructure;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;

public class V3_EpsilonSuperstructureKinematics {

  // -------------------------- DH Parameters -------------------------
  // [0       0       -l1     0]
  // [pi/2    0       0       0]
  // [0       l3+d3   0       0]
  // [0       0       0       pi/2]
  // [0       l5      0       0]
  // [theta5  0       0       -pi/2]
  // [-pi/2   l7      0       0]

  // -------- Homogeneous Transform from Base to End Effector ---------
  // [1,      0,              0,              -l1 + l5]
  // [0,      cos(theta5),    -sin(theta5),   -l7*sin(theta5)]
  // [0,      sin(theta5),    cos(theta5),    d3 + l3 + l7*cos(theta5)]
  // [0,      0,              0,              1]

  private static final double l1;
  private static final double l3;
  private static final double l5;
  private static final double l7;

  static {
    l1 = Units.inchesToMeters(7.0);
    l3 = Units.inchesToMeters(11.351122);
    l5 = Units.inchesToMeters(7.0);
    l7 = Units.inchesToMeters(22.259494);
  }

  private static final Matrix<N4, N4> fkMat(double d3, Rotation2d theta5) {
    double cos5 = theta5.getCos();
    double sin5 = theta5.getSin();

    return MatBuilder.fill(
        Nat.N4(),
        Nat.N4(),
        1,
        0,
        0,
        -l1 + l5,
        0,
        cos5,
        -sin5,
        -l7 * sin5,
        0,
        sin5,
        cos5,
        d3 + l3 + l7 * cos5,
        0,
        0,
        0,
        1);
  }

  /**
   * (Forward Kinematics) Calculates the end-effector's world coordinates.
   *
   * @return A Translation2d representing the (x, y) world coordinate.
   */
  public static EndEffectorPose fk(double d3, Rotation2d theta5) {
    Matrix<N4, N4> t = fkMat(d3, theta5);
    // Note: The FK matrix seems to be in the YZ plane based on the ik function signature.
    // We'll return the y and z components.
    return new EndEffectorPose(t.get(1, 3), t.get(2, 3));
  }

  /**
   * (Inverse Kinematics) Calculates the required joint states for a target coordinate.
   *
   * @param py The target y-position in the robot's frame.
   * @param pz The target z-position in the robot's frame.
   * @return A JointSolution containing the required d3 and theta5 values.
   */
  public static JointSolution ik(double py, double pz) {
    Rotation2d theta5 =
        Rotation2d.fromRadians(Math.atan2(Math.sqrt(1 - Math.pow((py / l7), 2)), py / l7))
            .minus(Rotation2d.kCCW_Pi_2);
    double d3 =
        pz - l3 - (l7 * Math.sin(Math.atan2(Math.sqrt(1 - Math.pow((py / l7), 2)), py / l7)));

    return new JointSolution(d3, theta5);
  }

  public static record EndEffectorPose(double y, double z) {}

  public static record JointSolution(double d3, Rotation2d theta5) {}
}
