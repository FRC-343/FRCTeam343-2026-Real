// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.TurretStuff;

import java.util.TreeMap;

/**
 * Lookup table for hub shooting parameters based on distance. Supports linear interpolation between
 * data points.
 */
public class HubLookUpTable {

  /** Data structure to hold shooting parameters */
  public static class ShootingParameters {
    public final double shooterSpeed; // RPS (Revolutions Per Second)
    public final double trajectoryAngle; // Degrees
    public final double timeOfFlight; // Seconds

    public ShootingParameters(double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
      this.shooterSpeed = shooterSpeed;
      this.trajectoryAngle = trajectoryAngle;
      this.timeOfFlight = timeOfFlight;
    }
  }

  // TreeMap automatically sorts by distance (key)
  private final TreeMap<Double, ShootingParameters> lookupTable;

  public HubLookUpTable() {
    lookupTable = new TreeMap<>();
    initializeLookupTable();
  }

  /** Initialize the lookup table with known data points */
  private void initializeLookupTable() {
    // Distance (m), Shooter Speed (RPS), Hood Angle (°), Time of Flight (s)
    // KrakenX60 shooting 226g ball - optimized for constant RPS ~75
    // Trajectory angles: 90° = straight up, 45° = maximum distance
    addEntry(0.9748297288085864, 31.0, 1.0, 1.31); // Close shot - nearly straight up
    addEntry(1.1905709983466113, 31.0, 1.0, 1.38);
    addEntry(1.5840514182961944, 31.0, 10, 1.31);
    addEntry(1.7479521465019607, 34.0, 6.0, 1.08);
    addEntry(1.8657662309629168, 36.0, 9, 1.35);
    addEntry(2.085376349442688, 38.0, 11, 1.18);
    addEntry(2.1877033944697533, 38.0, 12, 1.21);
    addEntry(2.2215721812153264, 38.0, 12.0, 1.21);
    addEntry(2.3474931867531477, 38.0, 12.0, 1.30);
    addEntry(2.5882889973048684, 39.0, 12.0, 1.28);
    addEntry(3.5019981710553605, 46.0, 18.0, 1.28);
    addEntry(4.807723436360055, 59.0, 28.0, 1.46);

    // Max distance - lowest angle
  }

  /** Add an entry to the lookup table */
  public void addEntry(
      double distance, double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
    lookupTable.put(distance, new ShootingParameters(shooterSpeed, trajectoryAngle, timeOfFlight));
  }

  /**
   * Get interpolated shooting parameters for a given distance
   *
   * @param distance Distance to target in meters
   * @return Interpolated shooting parameters
   */
  public ShootingParameters getParameters(double distance) {
    // Check if exact match exists
    if (lookupTable.containsKey(distance)) {
      return lookupTable.get(distance);
    }

    // Get the surrounding values
    Double lowerKey = lookupTable.floorKey(distance);
    Double upperKey = lookupTable.ceilingKey(distance);

    // Handle edge cases
    if (lowerKey == null) {
      return lookupTable.get(upperKey); // Below minimum distance
    }
    if (upperKey == null) {
      return lookupTable.get(lowerKey); // Above maximum distance
    }

    // Perform linear interpolation
    ShootingParameters lower = lookupTable.get(lowerKey);
    ShootingParameters upper = lookupTable.get(upperKey);

    double ratio = (distance - lowerKey) / (upperKey - lowerKey);

    double interpolatedSpeed = lerp(lower.shooterSpeed, upper.shooterSpeed, ratio);
    double interpolatedAngle = lerp(lower.trajectoryAngle, upper.trajectoryAngle, ratio);
    double interpolatedTime = lerp(lower.timeOfFlight, upper.timeOfFlight, ratio);

    return new ShootingParameters(interpolatedSpeed, interpolatedAngle, interpolatedTime);
  }

  /** Linear interpolation helper */
  private double lerp(double start, double end, double ratio) {
    return start + (end - start) * ratio;
  }

  /** Get shooter speed for a given distance */
  public double getShooterSpeed(double distance) {
    return getParameters(distance).shooterSpeed;
  }

  /** Get trajectory angle for a given distance */
  public double getTrajectoryAngle(double distance) {
    return getParameters(distance).trajectoryAngle;
  }

  /** Get time of flight for a given distance */
  public double getTimeOfFlight(double distance) {
    return getParameters(distance).timeOfFlight;
  }
}
