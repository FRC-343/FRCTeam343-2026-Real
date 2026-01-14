package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

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
}
