package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

public class TrajectoryGenerator {
  /**
   * Generates the 6 coefficients for a quintic polynomial trajectory.
   *
   * @param x_i Initial position
   * @param v_i Initial velocity
   * @param a_i Initial acceleration
   * @param x_f Final position
   * @param v_f Final velocity
   * @param a_f Final acceleration
   * @param t_i Initial time
   * @param t_f Final time
   * @return A 6x1 column vector of the polynomial coefficients (c₀ to c₅).
   */
  public static final Matrix<N6, N1> generateQuinticTrajectory(
      double x_i,
      double v_i,
      double a_i,
      double x_f,
      double v_f,
      double a_f,
      double t_i,
      double t_f) {
    Matrix<N6, N6> A =
        new Matrix<>(
            Nat.N6(),
            Nat.N6(),
            new double[] {
              1,
              t_i,
              Math.pow(t_i, 2),
              Math.pow(t_i, 3),
              Math.pow(t_i, 4),
              Math.pow(t_i, 5),
              0,
              1,
              2 * t_i,
              3 * Math.pow(t_i, 2),
              4 * Math.pow(t_i, 3),
              5 * Math.pow(t_i, 4),
              0,
              0,
              2,
              6 * t_i,
              12 * Math.pow(t_i, 2),
              20 * Math.pow(t_i, 3),
              1,
              t_f,
              Math.pow(t_f, 2),
              Math.pow(t_f, 3),
              Math.pow(t_f, 4),
              Math.pow(t_f, 5),
              0,
              1,
              2 * t_f,
              3 * Math.pow(t_f, 2),
              4 * Math.pow(t_f, 3),
              5 * Math.pow(t_f, 4),
              0,
              0,
              2,
              6 * t_f,
              12 * Math.pow(t_f, 2),
              20 * Math.pow(t_f, 3)
            });

    // The vector of boundary conditions
    Matrix<N6, N1> B =
        new Matrix<>(Nat.N6(), Nat.N1(), new double[] {x_i, v_i, a_i, x_f, v_f, a_f});

    return A.solve(B);
  }

  /**
   * Evaluates the position at a given time using the calculated trajectory coefficients.
   *
   * @param coeffs The 6x1 coefficient vector from generateQuinticTrajectory.
   * @param t The time at which to evaluate the position.
   * @return The position at time t.
   */
  public static final double evaluateQuinticTrajectory(Matrix<N6, N1> coeffs, double t) {
    return coeffs.get(0, 0)
        + coeffs.get(1, 0) * t
        + coeffs.get(2, 0) * Math.pow(t, 2)
        + coeffs.get(3, 0) * Math.pow(t, 3)
        + coeffs.get(4, 0) * Math.pow(t, 4)
        + coeffs.get(5, 0) * Math.pow(t, 5);
  }
}
