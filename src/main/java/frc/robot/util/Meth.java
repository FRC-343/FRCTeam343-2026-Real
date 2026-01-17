package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class Meth {

  public final class TurretAim {

    /** Returns turret yaw (field-relative). */
    public static double calculateYaw(
        Translation2d shooterPos,
        Translation2d robotVel,
        Translation2d targetPos,
        Translation2d targetVel,
        double time) {

      Translation2d intercept = targetPos.plus(targetVel.minus(robotVel).times(time));

      Translation2d delta = intercept.minus(shooterPos);

      return Math.atan2(delta.getY(), delta.getX());
    }
  }

  public final class HoodAim {

    /** Calculates hood angle in radians. Returns NaN if unreachable. */
    public static double calculateHoodAngle(
        double horizontalDistance, double heightDifference, double exitVelocity) {
      if (horizontalDistance < 0.05) return Double.NaN;
      if (exitVelocity < 0.1) return Double.NaN;

      double g = 9.81;
      double v2 = exitVelocity * exitVelocity;

      double term =
          v2 * v2 - g * (g * horizontalDistance * horizontalDistance + 2 * heightDifference * v2);

      if (term < 0) return Double.NaN;

      double sqrt = Math.sqrt(term);

      // Prefer low arc, fall back to high arc
      double low = Math.atan((v2 - sqrt) / (g * horizontalDistance));

      if (!Double.isNaN(low) && low > 0) return low;

      return Math.atan((v2 + sqrt) / (g * horizontalDistance));
    }
  }

  /**
   * Handles limited-rotation turret optimization (ex: 270° sweep). Works for real turrets OR
   * "robot-as-turret".
   */
  public final class TurretYawLimiter {

    // Example: 270° total travel (-135° to +135° relative to robot forward)

    // Optional soft margin to avoid hard stops
    public static final double SOFT_MARGIN_RAD = Math.toRadians(0);

    private TurretYawLimiter() {}

    /**
     * @param fieldYawRad Desired yaw in FIELD coordinates (from solver)
     * @param robotYawRad Current robot heading (gyro / pose)
     * @param currentTurretRad Current turret angle relative to robot
     * @return Best legal turret yaw (robot-relative), or NaN if unreachable
     */
    public static double optimizeYaw(
        double fieldYawRad, double robotYawRad, double currentTurretRad) {

      // Convert FIELD yaw → ROBOT-relative yaw
      double desiredRobotYaw = MathUtil.angleModulus(fieldYawRad - robotYawRad);

      // Generate equivalent angles (wrap handling)
      double[] candidates =
          new double[] {
            desiredRobotYaw, desiredRobotYaw + 2.0 * Math.PI, desiredRobotYaw - 2.0 * Math.PI
          };

      double bestYaw = Double.NaN;
      double bestCost = Double.POSITIVE_INFINITY;

      for (double candidate : candidates) {

        // Enforce hard + soft limits
        if (candidate < Constants.TurretConstants.TURRET_MIN_RAD + SOFT_MARGIN_RAD) continue;
        if (candidate > Constants.TurretConstants.TURRET_MAX_RAD - SOFT_MARGIN_RAD) continue;

        // Cost = smallest movement from current turret angle
        double cost = Math.abs(MathUtil.angleModulus(candidate - currentTurretRad));

        if (cost < bestCost) {
          bestCost = cost;
          bestYaw = candidate;
        }
      }

      return bestYaw; // NaN means "no valid solution"
    }
  }
}
