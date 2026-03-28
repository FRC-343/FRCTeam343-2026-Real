package frc.robot.bobot_state2.varc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.bobot_state2.BobotState;

public class TestTargetTracker extends TargetAngleTracker {
  private double distanceMeters = 0;
  private Rotation2d rotationTarget = Rotation2d.kZero;

  public void update() {
    Pose2d closestPose =
        Constants.TargetLocations.getHubTarget()
            .rotateBy(BobotState.getGlobalPose().getTranslation().getAngle());

    distanceMeters =
        closestPose.getTranslation().getDistance(BobotState.getGlobalPose().getTranslation());

    rotationTarget = closestPose.getRotation().plus(Rotation2d.kPi);
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public double getDistanceMeters() {
    return distanceMeters;
  }
}
