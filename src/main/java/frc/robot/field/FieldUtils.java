package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.bobot_state2.BobotState;
import java.util.List;

public class FieldUtils {
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
  }

  public static boolean isBlueAlliance() {
    return FieldUtils.getAlliance() == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return FieldUtils.getAlliance() == Alliance.Red;
  }

  public static double getFlipped() {
    return FieldUtils.isRedAlliance() ? -1 : 1;
  }

  public static Pose2d getHub() {
    return FieldUtils.isBlueAlliance()
        ? new Pose2d(
            FieldConstants.BLUE_HUB_CENTER_X, FieldConstants.HUB_CENTER_Y, new Rotation2d())
        : new Pose2d(
            FieldConstants.RED_HUB_CENTER_X, FieldConstants.HUB_CENTER_Y, new Rotation2d());
  }

  public static Pose2d getLeftTarget() {
    return FieldUtils.isBlueAlliance()
        ? new Pose2d(FieldConstants.BLUE_PASS_X, FieldConstants.BLUE_LEFT_PASS_Y, new Rotation2d())
        : new Pose2d(FieldConstants.RED_PASS_X, FieldConstants.RED_LEFT_PASS_Y, new Rotation2d());
  }

  public static Pose2d getRightTarget() {
    return FieldUtils.isBlueAlliance()
        ? new Pose2d(FieldConstants.BLUE_PASS_X, FieldConstants.BLUE_RIGHT_PASS_Y, new Rotation2d())
        : new Pose2d(FieldConstants.RED_PASS_X, FieldConstants.RED_RIGHT_PASS_Y, new Rotation2d());
  }

  public static HubFace getClosestHub() {
    List<HubFace> HubTags =
        FieldUtils.isBlueAlliance() ? FieldConstants.BLUEHUBTAGS : FieldConstants.REDHUBTAGS;
    Translation2d robotTranslation = BobotState.getGlobalPose().getTranslation();

    HubFace closestHub =
        HubTags.stream()
            .reduce(
                (HubFace hub1, HubFace hub2) ->
                    robotTranslation.getDistance(hub1.tag.pose().getTranslation().toTranslation2d())
                            < robotTranslation.getDistance(
                                hub2.tag.pose().getTranslation().toTranslation2d())
                        ? hub1
                        : hub2)
            .get();

    return closestHub;
  }

  // public static HPSFace getClosestHPSTag() {
  //   List<HPSFace> hpsTags =
  //       FieldUtils.isBlueAlliance() ? FieldConstants.blueHPSTags : FieldConstants.redHPSTags;

  //   Translation2d robotTranslation = BobotState.getGlobalPose().getTranslation();

  //   HPSFace closestTag =
  //       hpsTags.stream()
  //           .reduce(
  //               (HPSFace HPS1, HPSFace HPS2) ->
  //
  // robotTranslation.getDistance(HPS1.tag.pose().getTranslation().toTranslation2d())
  //                           < robotTranslation.getDistance(
  //                               HPS2.tag.pose().getTranslation().toTranslation2d())
  //                       ? HPS1
  //                       : HPS2)
  //           .get();

  //   return closestTag;
  // }

  // public static ProcessorFace getProcessorFace() {
  //   ProcessorFace processorTags =
  //       FieldUtils.isBlueAlliance() ? FieldConstants.blueProcessor : FieldConstants.redProcessor;
  //   ProcessorFace closestTag = processorTags;

  //   return closestTag;
  // }

  // public static BargeCage getBargeTag() {
  //   BargeCage test =
  //       FieldUtils.isBlueAlliance() ? FieldConstants.blueBarge : FieldConstants.redBarge;
  //   return FieldUtils.isBlueAlliance() ? FieldConstants.blueBarge : FieldConstants.redBarge;
  // }
}
