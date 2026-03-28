// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldUtils;
import frc.robot.field.HubFaces;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class HoodConstants {
    public static double MINHOOD = 0.1;
    public static double MAXHOOD = 34.3;
  }

  public static final class TurretConstants {

    public static final double FORWARDLIMITDEGREES = 180;
    public static final double REVERSELIMITDEGREES = -180;
    public static final double MOTOR_TO_TURRET_RATIO = 7.0 * 3.0;

    public static final double OFFSETMINUS = .03;

    public static final double TURRET_OFFSET_X = Units.inchesToMeters(-2.25);
    public static final double TURRET_OFFSET_Y = Units.inchesToMeters(4.625);
  }

  public static final class ShooterConstants {
    public static double WHEELDIAM = 4; // 4 inches
    public static double WHEELDIAMMETER = Units.inchesToMeters(WHEELDIAM);

    public static double CIRCUMFERENCE = Math.PI * WHEELDIAMMETER;
  }

  public static final class TargetLocations {
    public static Pose2d getHubTarget() {
      double x =
          HubFaces.B.get()
              .tag
              .pose()
              .getTranslation()
              .toTranslation2d()
              .plus(
                  new Translation2d(
                      FieldUtils.isBlueAlliance()
                          ? FieldConstants.tagToHub
                          : -FieldConstants.tagToHub,
                      0.0))
              .getX();
      double y =
          HubFaces.B.get()
              .tag
              .pose()
              .getTranslation()
              .toTranslation2d()
              .plus(
                  new Translation2d(
                      FieldUtils.isBlueAlliance()
                          ? FieldConstants.tagToHub
                          : -FieldConstants.tagToHub,
                      0.0))
              .getY();

      return new Pose2d(x, y, HubFaces.B.get().tag.pose().getRotation().toRotation2d());
    }

    public static Pose2d getBottomTarget() {
      double x = FieldUtils.isRedAlliance() ? 14.78 : .91;
      double y = FieldUtils.isRedAlliance() ? 1.72 : 1.72;
      return new Pose2d(x, y, HubFaces.B.get().tag.pose().getRotation().toRotation2d());
    }

    public static Pose2d getTopTarget() {
      double x = FieldUtils.isRedAlliance() ? 14.78 : .91;
      double y = FieldUtils.isRedAlliance() ? 6.46 : 6.46;
      return new Pose2d(x, y, HubFaces.B.get().tag.pose().getRotation().toRotation2d());
    }
  }
}
