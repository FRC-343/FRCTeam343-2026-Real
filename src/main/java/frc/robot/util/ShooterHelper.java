package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class ShooterHelper {

  public final class HoodAimTest {

    public static double requiredVelocity(
        double distance, double heightDifference, double hoodDegrees) {

      double g = 9.81;

      double theta = Math.toRadians(hoodDegrees);

      double cos = Math.cos(theta);
      double tan = Math.tan(theta);

      double numerator = g * distance * distance;
      double denominator = 2 * cos * cos * (distance * tan - heightDifference);

      if (denominator <= 0) return Double.NaN;

      return Math.sqrt(numerator / denominator);
    }

    public static double chooseHoodAngle(double distance) {

      // simple distance scaling
      double hood = 12 + distance * 4;

      hood = MathUtil.clamp(hood, Constants.HoodConstants.MINHOOD, Constants.HoodConstants.MAXHOOD);

      return hood;
    }
  }

  public class TimeOfFlightTest {

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
      double b = 2.0 * (rx * vx + ry * vy);
      double c = rx * rx + ry * ry;

      double disc = b * b - 4.0 * a * c;
      if (disc < 0) return Double.NaN;

      double sqrtD = Math.sqrt(disc);

      double t1 = (-b - sqrtD) / (2.0 * a);
      double t2 = (-b + sqrtD) / (2.0 * a);

      double t = Double.POSITIVE_INFINITY;

      if (t1 > 0.0) t = t1;
      if (t2 > 0.0 && t2 < t) t = t2;

      return Double.isFinite(t) ? t : Double.NaN;
    }
  }

  public final class TurretCalc {

    public static double calculateTurretSetpointRadians(
        Translation2d fieldTarget, Pose2d robotPose) {

      Translation2d robotToTarget = fieldTarget.minus(robotPose.getTranslation());

      Rotation2d fieldAngle = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());

      Rotation2d robotRelative = fieldAngle.minus(robotPose.getRotation());

      return robotRelative.getRadians();
    }

    public static double turretRadiansToMotorRotations(double turretRadians) {

      return (turretRadians / (2.0 * Math.PI)) * Constants.TurretConstants.MOTOR_TO_TURRET_RATIO;
    }

    public static double motorRotationsToTurretRadians(double motorRotations) {

      return (motorRotations / Constants.TurretConstants.MOTOR_TO_TURRET_RATIO) * (2 * Math.PI);
    }
  }

  public final class ShooterSpeed {
    public static double ShooterRPS(double distance) {
      double rps = 34.0 + (25.0 / 7.2) * (distance - 2.8);
      return rps;
    }
  }
}
