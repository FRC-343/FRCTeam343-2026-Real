package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class ShooterHelper {

  public final class TurretAim {

    /** Returns turret yaw (field-relative). */
    public static double calculateYaw(
        Translation2d shooterPos,
        Translation2d robotVel,
        Translation2d targetPos,
        Translation2d targetVel,
        double time) {

      Translation2d intercept = targetPos.plus(targetVel.minus(robotVel));

      Translation2d delta = intercept.minus(shooterPos);

      return Math.atan2(delta.getY(), delta.getX());
    }
  }

  public final class HoodAim {

    /** Calculates hood angle in radians. Returns NaN if unreachable. */
    public static double calculateHoodAngle(
        double horizontalDistance, double heightDifference, double exitVelocity) {
      if (horizontalDistance < 0.05)
        return Double.NaN;
      if (exitVelocity < 0.1)
        return Double.NaN;

      // term of gravity
      double g = 9.81;
      // velocity squared term
      double v2 = exitVelocity * exitVelocity;

      /*
       * more self explanatory than was previously, simulates parabolic arc of grav on
       * ball
       * and returns the y-position
       */
      double SimGravOnBall = v2 * v2 - g * (g * horizontalDistance * horizontalDistance + 2 * heightDifference * v2);

      // if the y-position of the ball is less than 0 (below ground), return NaN
      if (SimGravOnBall < 0)
        return Double.NaN;

      // self-explanatory
      double SqrtOfGravOnBall = Math.sqrt(SimGravOnBall);

      // Prefer low arc, fall back to high arc
      double low = (Math.atan((v2 - SqrtOfGravOnBall) / (g * horizontalDistance)) / (2 * Math.PI)) * 344.0;

      // if low exists and is more than 0, returns the low arc
      if (!Double.isNaN(low) && low > 0)
        return low;

      // returns the calculated high angle if low fails
      return (Math.atan((v2 + SqrtOfGravOnBall) / (g * horizontalDistance)) / (2 * Math.PI)) * 344.0;
    }
  }

  /**
   * Handles limited-rotation turret optimization (ex: 270° sweep). Works for real
   * turrets OR
   * "robot-as-turret".
   */
  public final class TurretYawLimiter {

    // Example: 270° total travel (-135° to +135° relative to robot forward)

    // Optional soft margin to avoid hard stops
    public static final double SOFT_MARGIN_RAD = Math.toRadians(5);

    private TurretYawLimiter() {
    }

    /**
     * @param fieldYawRad      Desired yaw in FIELD coordinates (from solver)
     * @param robotYawRad      Current robot heading (gyro / pose)
     * @param currentTurretRad Current turret angle relative to robot
     * @return Best legal turret yaw (robot-relative), or NaN if unreachable
     */
    public static double optimizeYaw(
        double fieldYawRad, double robotYawRad, double currentTurretRad) {

      // Convert FIELD yaw → ROBOT-relative yaw
      double desiredRobotYaw = MathUtil.angleModulus(fieldYawRad - robotYawRad);

      // Generate equivalent angles (wrap handling)
      double[] candidates = new double[] {
          desiredRobotYaw, desiredRobotYaw + 2.0 * Math.PI, desiredRobotYaw - 2.0 * Math.PI
      };

      double bestYaw = Double.NaN;
      double bestCost = Double.POSITIVE_INFINITY;

      for (double candidate : candidates) {

        // Enforce hard + soft limits
        if (candidate < Constants.TurretConstants.REVERSELIMITDEGREES + SOFT_MARGIN_RAD)
          continue;
        if (candidate > Constants.TurretConstants.FORWARDLIMITDEGREES - SOFT_MARGIN_RAD)
          continue;

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

  public class TimeOfFlight {

    /**
     * Solves time of intercept in the XY plane. Returns NaN if no solution exists.
     */
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
      if (disc < 0)
        return Double.NaN;

      double sqrtD = Math.sqrt(disc);
      double t1 = (-b - sqrtD) / (2 * a);
      double t2 = (-b + sqrtD) / (2 * a);

      double t = Double.POSITIVE_INFINITY;
      if (t1 > 0)
        t = t1;
      if (t2 > 0 && t2 < t)
        t = t2;

      return Double.isFinite(t) ? t : Double.NaN;
    }
  }

  public final class TurretCRT2 {
    public static double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {
      double difference = e2 - e1;
      if (difference > 250) {
        difference -= 360;
      }
      if (difference < -250) {
        difference += 360;
      }

      difference *= TurretConstants.SLOPE;

      double e1Rotations = (difference * TurretConstants.GEAR_0_TOOTH_COUNT / TurretConstants.GEAR_1_TOOTH_COUNT)
          / 360.0;
      double e1RotationsFloored = Math.floor(e1Rotations);
      double turretAngle = (e1RotationsFloored * 360.0 + e1)
          * (TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT);
      if (turretAngle - difference < -100) {
        turretAngle += TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT * 360.0;
      } else if (turretAngle - difference > 100) {
        turretAngle -= TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT * 360.0;
      }
      return Units.degreesToRotations(turretAngle);
    }

    public static double calculateTurretSetpointRadians(
        Translation2d fieldTarget, Pose2d robotPose, Rotation2d currentTurretAngle) {

      Translation2d robotToTarget = fieldTarget.minus(robotPose.getTranslation());

      Rotation2d fieldAngle = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());

      Rotation2d robotRelative = fieldAngle.minus(robotPose.getRotation());

      double baseTargetRad = robotRelative.getRadians();

      return baseTargetRad;
    }

    public static double turretRadiansToMotorRotations(double turretRadians) {
      return (turretRadians / (2 * Math.PI)) * TurretConstants.MOTOR_TO_TURRET_RATIO;
    }

    public static double motorRotationsToTurretRadians(double motorRotations) {
      return (motorRotations / TurretConstants.MOTOR_TO_TURRET_RATIO) * (2 * Math.PI);
    }
  }

  public final class TurretCRT3 {
    public static double calculateTurretRotations(double e1ValDegrees, double e2ValDegrees) {

      int e1Teeth = 13;
      int e2Teeth = 17;
      int tTeeth = 221;

      double bestMatch = -1;
      double tolerance = 1e-4;

      // n1 goes 0 to e2Teeth - 1
      for (int n1 = 0; n1 < e2Teeth; n1++) {

        double turretFromE1 = (n1 + e1ValDegrees / 360.0) * e1Teeth / tTeeth;

        // n2 goes 0 to e1Teeth - 1
        for (int n2 = 0; n2 < e1Teeth; n2++) {

          double turretFromE2 = (n2 + e2ValDegrees / 360.0) * e2Teeth / tTeeth;

          if (Math.abs(turretFromE1 - turretFromE2) < tolerance) {
            bestMatch = turretFromE1;
            return bestMatch;
          }
        }
      }

      return -1; // no solution found (should not happen if gears are co-prime)
    }

    /** Returns turret aiming angle (0–360 degrees). */
    public static double getTurretAimAngle(double turretRotations) {
      double fractional = turretRotations % 1.0;
      if (fractional < 0) {
        fractional += 1.0;
      }
      return fractional * 360.0;
    }
  }
}
