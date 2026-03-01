// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

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
    public static double minHood = 0.1;
    public static double maxHood = 34.3;
  }

  public static final class TurretConstants {

    public static final double FORWARDLIMITDEGREES = 180;
    public static final double REVERSELIMITDEGREES = -180;
    public static final double MOTOR_TO_TURRET_RATIO = 221 / 21 * 3;

    // ===== Gear Math =====
    public static final int CRT_MOD_A = 17;
    public static final int CRT_MOD_B = 13;
    public static final int CRT_PERIOD = 221; // 17 * 13

    // Motor rotations per turret rotation
    public static final double MOTOR_ROT_PER_TURRET_ROT = 221.0 / 7.0;

    public static final double RAD_PER_TURRET_ROT = 2.0 * Math.PI;

    public static final double GEAR_0_TOOTH_COUNT = 221.0;
    public static final double GEAR_1_TOOTH_COUNT = 17.0;
    public static final double GEAR_2_TOOTH_COUNT = 13.0;

    public static final double SLOPE =
        ((GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
            / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT));
  }

  public static final class ShooterConstants {
    public static double WHEELDIAM = 4; // 4 inches
    public static double WHEELDIAMMETER = Units.inchesToMeters(WHEELDIAM);

    public static double CIRCUMFERENCE = Math.PI * WHEELDIAMMETER;
  }
}
