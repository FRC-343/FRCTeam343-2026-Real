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
    public static double MINHOOD = 0.1;
    public static double MAXHOOD = 34.3;
  }

  public static final class TurretConstants {

    public static final double FORWARDLIMITDEGREES = 180.0;
    public static final double REVERSELIMITDEGREES = -180.0;
    public static final double MOTOR_TO_TURRET_RATIO = 7.0 * 3.0;

    public static final double TURRET_OFFSET_X = Units.inchesToMeters(-2.25);
    public static final double TURRET_OFFSET_Y = Units.inchesToMeters(4.8); // 4.8125 normal
  }
}
