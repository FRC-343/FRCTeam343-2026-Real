package frc.robot.Thisjustatestfr;

import edu.wpi.first.math.geometry.Translation3d;

public class AutoAim {
  public final class FrcBallisticTurret {

    private static final double GRAVITY = 9.81; // m/s^2

    public static class AimSolution {
      public final double yawRadians;
      public final double hoodRadians;
      public final double flightTime;

      public AimSolution(double yawRadians, double hoodRadians, double flightTime) {
        this.yawRadians = yawRadians;
        this.hoodRadians = hoodRadians;
        this.flightTime = flightTime;
      }
    }

    /**
     * @param shooterPos Shooter position on field (meters)
     * @param robotVel Robot chassis velocity (m/s)
     * @param targetPos Target position on field (meters)
     * @param targetVel Target velocity (usually zero)
     * @param exitVelocity Shooter exit velocity (m/s)
     * @return AimSolution or null if no physical solution exists
     */
    public static AimSolution calculateAim(
        edu.wpi.first.math.geometry.Translation3d shooterPos,
        edu.wpi.first.math.geometry.Translation3d robotVel,
        edu.wpi.first.math.geometry.Translation3d targetPos,
        edu.wpi.first.math.geometry.Translation3d targetVel,
        double exitVelocity) {

      // Relative motion (target as seen from robot)
      Translation3d relPos = targetPos.minus(shooterPos);
      Translation3d relVel = targetVel.minus(robotVel);

      // Solve intercept time in horizontal plane (X-Y)
      double rx = relPos.getX();
      double ry = relPos.getY();
      double vx = relVel.getX();
      double vy = relVel.getY();

      double a = vx * vx + vy * vy - exitVelocity * exitVelocity;
      double b = 2 * (rx * vx + ry * vy);
      double c = rx * rx + ry * ry;

      double disc = b * b - 4 * a * c;
      if (disc < 0) return null;

      double sqrtDisc = Math.sqrt(disc);
      double t1 = (-b - sqrtDisc) / (2 * a);
      double t2 = (-b + sqrtDisc) / (2 * a);

      double t = Double.POSITIVE_INFINITY;
      if (t1 > 0) t = t1;
      if (t2 > 0 && t2 < t) t = t2;
      if (!Double.isFinite(t)) return null;

      // Predict intercept point
      Translation3d intercept = targetPos.plus(relVel.times(t));

      Translation3d delta = intercept.minus(shooterPos);

      // Turret yaw (field-relative)
      double yaw = Math.atan2(delta.getY(), delta.getX());

      // Horizontal distance
      double distanceXY = Math.hypot(delta.getX(), delta.getY());
      double height = delta.getZ();

      // Ballistic hood angle
      double v2 = exitVelocity * exitVelocity;
      double g = GRAVITY;

      double term = v2 * v2 - g * (g * distanceXY * distanceXY + 2 * height * v2);
      if (term < 0) return null;

      // Use LOW arc for FRC (faster, more consistent)
      double hood = Math.atan((v2 - Math.sqrt(term)) / (g * distanceXY));

      return new AimSolution(yaw, hood, t);
    }
  }
}
