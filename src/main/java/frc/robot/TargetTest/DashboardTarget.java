package frc.robot.TargetTest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldUtils;
import frc.robot.field.HubFaces;

public class DashboardTarget {

  private final NetworkTableEntry xEntry; // Gets the X position of what we want on the field
  private final NetworkTableEntry yEntry; // Gets the Y position of what we want on the field

  public DashboardTarget() {
    NetworkTable table =
        NetworkTableInstance.getDefault()
            .getTable("dashboard"); // The Network Table we are wanting to pull information from

    xEntry = table.getEntry("targetX"); // The data for X
    yEntry = table.getEntry("targetY"); // The data for Y

    // Explicit default values so it doesnt error on start up
    // Or if the python program isnt open/buggs(on match start specifically) it doesnt screw us
    // xEntry.setDouble(0.0);
    // yEntry.setDouble(
    //     HubFaces.B.get()
    //         .tag
    //         .pose()
    //         .getTranslation()
    //         .toTranslation2d()
    //         .plus(
    //             new Translation2d(
    //                 FieldUtils.isBlueAlliance()
    //                     ? FieldConstants.tagToHub
    //                     : -FieldConstants.tagToHub,
    //                 0.0))
    //         .getY());
  }
  // Get method for our selected position
  public Translation2d getTarget() {
    double x =
        xEntry.getDouble(
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
                .getX());
    double y =
        yEntry.getDouble(
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
                .getY());
    return new Translation2d(x, y);
  }
}
