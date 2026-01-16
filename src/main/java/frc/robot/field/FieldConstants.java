package frc.robot.field;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision2.VisionConstants;
import java.util.List;

public class FieldConstants {
  /** AdvantageKit-safe loggable version of `AprilTag` that contains data we want without lookups */
  public static record AprilTagStruct(int fiducialId, Pose3d pose) {}

  /** Distance from the center of the April Tag on the Face to the center of the Pole */
  public static final double tagToHub = Units.inchesToMeters(23.5);

  public static final double distanceToTag = Units.inchesToMeters(10);

  public static final double tagToCageRight = Units.inchesToMeters(44.18);
  public static final double tagToCageCenter = Units.inchesToMeters(1.25);
  public static final double tagToCageLeft = Units.inchesToMeters(-41.7);

  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX = Units.inchesToMeters(299.438);

  /** Used for calculating HPS zoning */
  public static final double halfFieldWidth = fieldWidth / 2;

  /*
   * April Tag Lookup
   */
  //   public static final HPSFace blueHPSDriverRight =
  //       new HPSFace(new AprilTagStruct(12, VisionConstants.fieldLayout.getTagPose(12).get()));
  //   public static final HPSFace blueHPSDriverLeft =
  //       new HPSFace(new AprilTagStruct(13, VisionConstants.fieldLayout.getTagPose(13).get()));

  //   public static final ProcessorFace blueProcessor =
  //       new ProcessorFace(new AprilTagStruct(16,
  // VisionConstants.fieldLayout.getTagPose(16).get()));
  //   public static final BargeCage blueBarge =
  //       new BargeCage(new AprilTagStruct(14, VisionConstants.fieldLayout.getTagPose(14).get()));

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

  //   public static final HPSFace redHPSDriverLeft =
  //       new HPSFace(new AprilTagStruct(1, VisionConstants.fieldLayout.getTagPose(1).get()));
  //   public static final HPSFace redHPSDriverRight =
  //       new HPSFace(new AprilTagStruct(2, VisionConstants.fieldLayout.getTagPose(2).get()));

  //   public static final ProcessorFace redProcessor =
  //       new ProcessorFace(new AprilTagStruct(3,
  // VisionConstants.fieldLayout.getTagPose(3).get()));
  //   public static final BargeCage redBarge =
  //       new BargeCage(new AprilTagStruct(5, VisionConstants.fieldLayout.getTagPose(5).get()));

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

  //   public static final List<HPSFace> blueHPSTags = List.of(blueHPSDriverLeft,
  // blueHPSDriverRight);
  //   public static final List<HPSFace> redHPSTags = List.of(redHPSDriverLeft, redHPSDriverRight);
}
