package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.util.PoseUtils;

public class HubFace {
  public final AprilTagStruct tag;
  public final HubTags leftPole;
  public final HubTags rightPole;

  public HubFace(AprilTagStruct tag) {
    this.tag = tag;
    this.leftPole = new HubTags(tag);
    this.rightPole = new HubTags(tag);
  }

  public double getPerpendicularError(Pose2d robotPose) {
    return PoseUtils.getPerpendicularError(robotPose, tag.pose().toPose2d());
  }
}
