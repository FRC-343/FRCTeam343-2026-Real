package frc.robot.field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision2.VisionConstants;
import java.util.List;

public class FieldConstants {
  /** AdvantageKit-safe loggable version of `AprilTag` that contains data we want without lookups */
  public static record AprilTagStruct(int fiducialId, Pose3d pose) {}

  /** Distance from the center of the April Tag on the Face to the center of the Pole */

  // we hardcoding everything

  public static final double BLUE_HUB_CENTER_X = Units.inchesToMeters(182.11);

  public static final double RED_HUB_CENTER_X = Units.inchesToMeters(469.11);
  public static final double HUB_CENTER_Y = Units.inchesToMeters(164.845);

  public static final double BLUE_PASS_X = Units.inchesToMeters(91.055);
  public static final double RED_PASS_X = Units.inchesToMeters(560.165);
  public static final double BLUE_LEFT_PASS_Y = Units.inchesToMeters(234.775);
  public static final double BLUE_RIGHT_PASS_Y = Units.inchesToMeters(79.4225);
  public static final double RED_LEFT_PASS_Y = Units.inchesToMeters(234.775);
  public static final double RED_RIGHT_PASS_Y = Units.inchesToMeters(79.4225);

  public static final double tagToHub = Units.inchesToMeters(23.5);

  public static final double distanceToTag = Units.inchesToMeters(10);

  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX = Units.inchesToMeters(299.438);

  public static final double centerZoneWidth = Units.inchesToMeters(50);

  /** Used for calculating HPS zoning */
  public static final double halfFieldWidth = fieldWidth / 2;

  public static final double centerZoneWidthTop = halfFieldWidth + Units.inchesToMeters(20);
  public static final double centerZoneWidthBottom = halfFieldWidth - Units.inchesToMeters(20);

  public static final double distanceToBlueTrench = Units.inchesToMeters(186.1);
  public static final double distanceToRedTrench = fieldLength - (distanceToBlueTrench);

  // Blue Hub
  public static final HubFace blueHubA =
      new HubFace(new AprilTagStruct(25, VisionConstants.fieldLayout.getTagPose(25).get()));
  public static final HubFace blueHubB =
      new HubFace(new AprilTagStruct(26, VisionConstants.fieldLayout.getTagPose(26).get()));
  public static final HubFace blueHubC =
      new HubFace(new AprilTagStruct(27, VisionConstants.fieldLayout.getTagPose(27).get()));
  public static final HubFace blueHubD =
      new HubFace(new AprilTagStruct(18, VisionConstants.fieldLayout.getTagPose(18).get()));
  public static final HubFace blueHubE =
      new HubFace(new AprilTagStruct(19, VisionConstants.fieldLayout.getTagPose(19).get()));
  public static final HubFace blueHubF =
      new HubFace(new AprilTagStruct(20, VisionConstants.fieldLayout.getTagPose(20).get()));
  public static final HubFace blueHubG =
      new HubFace(new AprilTagStruct(21, VisionConstants.fieldLayout.getTagPose(21).get()));
  public static final HubFace blueHubH =
      new HubFace(new AprilTagStruct(24, VisionConstants.fieldLayout.getTagPose(24).get()));

  // Red Hub
  public static final HubFace redHubA =
      new HubFace(new AprilTagStruct(9, VisionConstants.fieldLayout.getTagPose(9).get()));
  public static final HubFace redHubB =
      new HubFace(new AprilTagStruct(10, VisionConstants.fieldLayout.getTagPose(10).get()));
  public static final HubFace redHubC =
      new HubFace(new AprilTagStruct(11, VisionConstants.fieldLayout.getTagPose(11).get()));
  public static final HubFace redHubD =
      new HubFace(new AprilTagStruct(2, VisionConstants.fieldLayout.getTagPose(2).get()));
  public static final HubFace redHubE =
      new HubFace(new AprilTagStruct(3, VisionConstants.fieldLayout.getTagPose(3).get()));
  public static final HubFace redHubF =
      new HubFace(new AprilTagStruct(4, VisionConstants.fieldLayout.getTagPose(4).get()));
  public static final HubFace redHubG =
      new HubFace(new AprilTagStruct(5, VisionConstants.fieldLayout.getTagPose(5).get()));
  public static final HubFace redHubH =
      new HubFace(new AprilTagStruct(8, VisionConstants.fieldLayout.getTagPose(8).get()));

  public static final List<HubFace> BLUEHUBTAGS = List.of(blueHubB, blueHubD, blueHubF, blueHubH);
  public static final List<HubFace> REDHUBTAGS = List.of(redHubB, redHubD, redHubF, redHubH);

  // public static final List<HPSFace> blueHPSTags = List.of(blueHPSDriverLeft,
  // blueHPSDriverRight);
  // public static final List<HPSFace> redHPSTags = List.of(redHPSDriverLeft,
  // redHPSDriverRight);
}
