package frc.robot.bobot_state2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state2.varc.TargetAngleTracker;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.vision2.PoseObservation;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import org.littletonrobotics.junction.Logger;

/**
 * Class full of static variables and methods that store robot state we'd need across mulitple
 * subsystems. It's called {@link #BobotState} as to not conflict with WPILib's {@link
 * edu.wpi.first.wpilibj.RobotState}
 */
public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = new Pose2d(); // Robots position on the field.

  private static boolean
      atWantedPerpPose; // Robots perpendicular position in relation to whatever Apriltag we are
  // lining up to.

  private static boolean
      atWantedRot; // Robots rotation in relation to whatever Apriltag we are lining up to

  private static boolean
      atWantedParaPose; // Robots parallel position in relation to whatever Apriltag we are lining
  // up to

  private static List<TargetAngleTracker> autoAlignmentTrackers = List.of();

  /*
   *  Adding new Tracking info below this
   *
   *  This will not have the update calls
   *  those will be added below the other update calls
   *  and it will have a section similar to this
   *
   */

  /*
   * Highlighting small section importaint for our shooter calcs
   */

  private static double
      ToF; // this will hold the Time of flight info needed for turret and hood calcs

  private static double HoodCalc; // the number that the hood calc spits out

  private static double TurretCalc; // the number that the turret calc spits out

  private static double ShooterVelo;

  /*
   * Highlighting small section importaint for our shooter calcs
   */

  private static double HoodPos; // this will store the hood position

  private static double TurretPos; // this will store the turret position

  private static double ShooterRPM;

  public static void updateWantedPose(boolean perpPoseWanted) {
    BobotState.atWantedPerpPose = perpPoseWanted;
  }

  public static void updateWantedParaPose(boolean paraPoseWanted) {
    BobotState.atWantedParaPose = paraPoseWanted;
  }

  public static void updateWantedRot(boolean rotWanted) {
    BobotState.atWantedRot = rotWanted;
  }

  public static void offerVisionObservation(PoseObservation observation) {
    BobotState.poseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getVisionObservations() {
    return BobotState.poseObservations;
  }

  public static void updateGlobalPose(Pose2d pose) {
    BobotState.globalPose = pose;
  }

  /*
   *
   * Section that we are adding updates too
   *
   */

  public static void updateToF(Double ToF) {
    BobotState.ToF = ToF;
  }

  public static void updateTurretYaw(double Yaw) {
    BobotState.TurretCalc = Yaw;
  }

  public static void updateHoodAngle(double Yaw) {
    BobotState.HoodCalc = Yaw;
  }

  public static void updateShooterVelo(double velo) {
    BobotState.ShooterVelo = velo;
  }

  public static void updateShooterRPM(Double RPM) {
    BobotState.ShooterRPM = RPM;
  }

  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  /*
   *
   * Section we are adding get methods to
   *
   */

  public static double getToF() {
    return BobotState.ToF;
  }

  public static double getShooterVelo() {
    return BobotState.ShooterVelo;
  }

  public static double getShooterRPM() {
    return BobotState.ShooterRPM;
  }

  public static double getTurretYaw() {
    return BobotState.TurretCalc;
  }

  public static Trigger onTeamSide() {
    return new Trigger(
        () ->
            FieldUtils.getAlliance() == Alliance.Blue
                ? getGlobalPose().getX() < FieldConstants.fieldLength / 2.0
                : getGlobalPose().getX() > FieldConstants.fieldLength / 2.0);
  }

  // public static TargetAngleTracker getClosestAlignmentTracker() {
  //   return autoAlignmentTrackers.stream()
  //       .reduce((a, b) -> a.getDistanceMeters() < b.getDistanceMeters() ? a : b)
  //       .get();
  // }

  @Override
  public void periodic() {

    Logger.recordOutput(logRoot + "Wanted Perp Pose", atWantedPerpPose);

    Logger.recordOutput(logRoot + "Wanted Para Pose", atWantedParaPose);

    Logger.recordOutput(logRoot + "Wanted Rot", atWantedRot);

    Logger.recordOutput(logRoot + "Time of Flight", ToF);

    Logger.recordOutput(logRoot + "Shooter Exit velo", ShooterVelo);

    Logger.recordOutput(logRoot + "Turret Wanted Yaw", TurretCalc);

    Logger.recordOutput(logRoot + "Hood Wanted Position", HoodCalc);

    Logger.recordOutput(logRoot + "RobotPose", globalPose);
    // {
    //   String calcLogRoot = logRoot + "ClosestAlignment/";
    //   Logger.recordOutput(
    //       calcLogRoot + "Type", getClosestAlignmentTracker().getClass().getSimpleName());
    // }
  }

  @Override
  public void simulationPeriodic() {}
}
