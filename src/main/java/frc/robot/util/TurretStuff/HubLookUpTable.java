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
    addEntry(1.2918881366663453, 30.0, 14.0, 1.01); // Close shot - nearly straight up
    addEntry(1.9090784403273822, 33.0, 19.0, 1.05);
    addEntry(2.3142050789213524, 34.0, 19, 1.20);
    addEntry(2.675989044663924, 36.0, 20.0, 1.31);
    addEntry(2.996659807285883, 37.0, 20.0, 1.20);
    addEntry(3.311555441242927, 39.0, 21.0, 1.21);
    addEntry(4.151147899829032, 41.0, 22.0, 1.41);
    addEntry(5.427651714921544, 42.0, 24.0, 1.35);

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
