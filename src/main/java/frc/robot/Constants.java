// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

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
    public static double minHood = 0.0;
    public static double maxHood = 0.0;
  }

  public static final class TurretConstants {
    public static final double TURRET_MIN_RAD = Math.toRadians(-150);
    public static final double TURRET_MAX_RAD = Math.toRadians(150);

    // Encoder rotations per turret rotation
    public static final double ENCODER_ROTATIONS_PER_TURRET_ROTATION =
        10.0; // this is a temp number

    // Derived constants
    public static final double TURRET_ROTATIONS_PER_ENCODER_ROTATION =
        1.0 / ENCODER_ROTATIONS_PER_TURRET_ROTATION;

    public static final double RADIANS_PER_TURRET_ROTATION = 2.0 * Math.PI;

    public static final double RADIANS_PER_ENCODER_ROTATION =
        RADIANS_PER_TURRET_ROTATION * TURRET_ROTATIONS_PER_ENCODER_ROTATION;
  }

  public static final class ShooterConstants {
    public static double WheelCir = 6;
  }
}
