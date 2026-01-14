package frc.robot.Thisjustatestfr;

import edu.wpi.first.math.geometry.Translation2d;

public class TimeOfFlight {

  /** Solves time of intercept in the XY plane. Returns NaN if no solution exists. */
  public static double solveTime(
      Translation2d shooterPos,
      Translation2d robotVel,
      Translation2d targetPos,
      Translation2d targetVel,
      double projectileSpeed) {

    Translation2d relPos = targetPos.minus(shooterPos);
    Translation2d relVel = targetVel.minus(robotVel);

    double rx = relPos.getX();
    double ry = relPos.getY();
    double vx = relVel.getX();
    double vy = relVel.getY();

    double a = vx * vx + vy * vy - projectileSpeed * projectileSpeed;
    double b = 2 * (rx * vx + ry * vy);
    double c = rx * rx + ry * ry;

    double disc = b * b - 4 * a * c;
    if (disc < 0) return Double.NaN;

    double sqrtD = Math.sqrt(disc);
    double t1 = (-b - sqrtD) / (2 * a);
    double t2 = (-b + sqrtD) / (2 * a);

    double t = Double.POSITIVE_INFINITY;
    if (t1 > 0) t = t1;
    if (t2 > 0 && t2 < t) t = t2;

    return Double.isFinite(t) ? t : Double.NaN;
  }
}
