package frc.robot.bobot_state2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.TargetLocations;
import frc.robot.bobot_state2.varc.HubTagTracker;
import frc.robot.bobot_state2.varc.TargetAngleTracker;
import frc.robot.bobot_state2.varc.TestTargetTracker;
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

  // adding Tag Trackers here
  private static HubTagTracker hubTracker = new HubTagTracker();
  private static TestTargetTracker TestTarget = new TestTargetTracker();

  // list of tags used to calculate where the hub is
  private static List<TargetAngleTracker> autoAlignmentTrackers = List.of(BobotState.hubTracker);

  /*
   * Adding new Tracking info below this
   *
   * This will not have the update calls
   * those will be added below the other update calls
   * and it will have a section similar to this
   *
   */

  /*
   * Highlighting small section importaint for our shooter calcs
   */

  private static double wantedRotRobot;

  private static double
      ToF; // this will hold the Time of flight info needed for turret and hood calcs

  private static double distance;

  private static double HoodCalc; // the number that the hood calc spits out

  private static double TurretCalc; // the number that the turret calc spits out

  private static double ShooterRPS; // Shooter RPS

  private static double ShooterWantedRPS; // Shooter Wanted RPS

  /*
   * Highlighting small section importaint for our shooter calcs
   */

  private static double HoodPos; // this will store the hood position

  private static double TurretPos1; // this will store the turret position

  private static double TurretPos2; // this will store the turret position

  private static double TurretMotorPos; // this will store the turret position

  private static double ShooterRPM; // Shooter RPM

  private static ChassisSpeeds roboChassisSpeeds; // Robot speed

  private static double OptiTurretYaw; // optimized turret angle

  private static Pose2d TurretTarget; // Turret target pose

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
  public static void updateDistance(double distance) {
    BobotState.distance = distance;
  }

  public static void updateWantedRobotRot(double rot) {
    BobotState.wantedRotRobot = rot;
  }

  // ToF = Time of Flight
  public static void updateToF(Double ToF) {
    BobotState.ToF = ToF;
  }

  public static void updateTurretYaw(double Yaw) {
    BobotState.TurretCalc = Yaw;
  }

  public static void updateHoodAngle(double Yaw) {
    BobotState.HoodCalc = Yaw;
  }

  public static void updateShooterRPS(double RPS) {
    BobotState.ShooterRPS = RPS;
  }

  public static void updateShooterRPM(Double RPM) {
    BobotState.ShooterRPM = RPM;
  }

  public static void updateWantedShooterRPS(double RPS) {
    BobotState.ShooterWantedRPS = RPS;
  }

  public static void updateTurretPos1(double pose) {
    BobotState.TurretPos1 = pose;
  }

  public static void updateTurretPos2(double pose) {
    BobotState.TurretPos2 = pose;
  }

  public static void updateTurretMotorPos(double pose) {
    BobotState.TurretMotorPos = pose;
  }

  public static void updateRoboChassisSpeed(ChassisSpeeds speed) {
    BobotState.roboChassisSpeeds = speed;
  }

  public static void updateOptiTurretYaw(double test) {
    BobotState.OptiTurretYaw = test;
  }

  public static void updateTurretTarget(Pose2d target) {
    BobotState.TurretTarget = target;
  }

  public static void updateHood(double pos) {
    BobotState.HoodPos = pos;
  }

  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  /*
   *
   * Section we are adding get methods to
   *
   */
  public static double getDistance() {
    return BobotState.distance;
  }

  public static double getToF() {
    return BobotState.ToF;
  }

  public static double getShooterRPS() {
    return BobotState.ShooterRPS;
  }

  public static double getShooterRPM() {
    return BobotState.ShooterRPM;
  }

  public static double getWantedShooterRPS() {
    return BobotState.ShooterWantedRPS;
  }

  public static double getTurretYaw() {
    return BobotState.TurretCalc;
  }

  public static ChassisSpeeds getRoboSpeed() {
    return BobotState.roboChassisSpeeds;
  }

  public static double getTurretPosi1() {
    return BobotState.TurretPos1;
  }

  public static double getTurretPosi2() {
    return BobotState.TurretPos2;
  }

  public static double getTurretMotorPosi() {
    return BobotState.TurretMotorPos;
  }

  public static double getOptiTurretYaw() {
    return BobotState.OptiTurretYaw;
  }

  public static double getWantedRobotRot() {
    return BobotState.wantedRotRobot;
  }

  public static Rotation2d getRotationtoClosestHub() {
    return BobotState.hubTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToTarget() {
    return BobotState.TestTarget.getRotationTarget();
  }

  public static Pose2d getTurretTarget() {
    return BobotState.TurretTarget;
  }

  public static double getHoodPos() {
    return BobotState.HoodPos;
  }

  public static double getWantedHood() {
    return BobotState.HoodCalc;
  }

  public static Pose2d targetLocation() {
    return (onTeamSide().getAsBoolean()
        ? TargetLocations.getHubTarget()
        : onTopHalf().getAsBoolean()
            ? TargetLocations.getTopTarget()
            : onBottomHalf().getAsBoolean()
                ? TargetLocations.getBottomTarget()
                : TargetLocations.getHubTarget());
  }

  // public static TargetAngleTracker getClosestAlignmentTracker() {
  // return autoAlignmentTrackers.stream()
  // .reduce((a, b) -> a.getDistanceMeters() < b.getDistanceMeters() ? a : b)
  // .get();
  // }

  // Adding Triggers here

  public static Trigger onBottomHalf() {
    return new Trigger(
        () -> getGlobalPose().getY() <= FieldConstants.centerZoneWidthBottom ? true : false);
  }

  public static Trigger onTopHalf() {
    return new Trigger(
        () -> getGlobalPose().getY() >= FieldConstants.centerZoneWidthTop ? true : false);
  }

  public static Trigger onTeamSide() {
    return new Trigger(
        () ->
            FieldUtils.getAlliance() == Alliance.Blue
                ? getGlobalPose().getX() < FieldConstants.distanceToBlueTrench // fix this
                : getGlobalPose().getX() > FieldConstants.distanceToRedTrench);
  }

  @Override
  public void periodic() {

    Logger.recordOutput(logRoot + "Wanted robot rot", wantedRotRobot);

    Logger.recordOutput(logRoot + "Wanted Perp Pose", atWantedPerpPose);

    Logger.recordOutput(logRoot + "Wanted Para Pose", atWantedParaPose);

    Logger.recordOutput(logRoot + "Wanted Rot", atWantedRot);

    Logger.recordOutput(logRoot + "Time of Flight", ToF);

    Logger.recordOutput(logRoot + "Shooter RPS", ShooterRPS);

    Logger.recordOutput(logRoot + "Shooter Wanted RPS", ShooterWantedRPS);

    Logger.recordOutput(logRoot + "Shooter RPM", ShooterRPM);

    Logger.recordOutput(logRoot + "Turret Wanted Yaw", TurretCalc);

    Logger.recordOutput(logRoot + "Hood Wanted Position", HoodCalc);

    Logger.recordOutput(logRoot + "RobotPose", globalPose);

    Logger.recordOutput(logRoot + "Turret Active Position 1", TurretPos1);

    Logger.recordOutput(logRoot + "Turret Active Position 2", TurretPos2);

    Logger.recordOutput(logRoot + "Robot Speed", roboChassisSpeeds);

    Logger.recordOutput(logRoot + "Opti Turret Yaw", OptiTurretYaw);

    Logger.recordOutput(logRoot + "Turret Target", TurretTarget);

    Logger.recordOutput(
        logRoot + "Turret Max limit", Constants.TurretConstants.FORWARDLIMITDEGREES);

    Logger.recordOutput(
        logRoot + "Turret Min limit", Constants.TurretConstants.REVERSELIMITDEGREES);

    Logger.recordOutput(logRoot + "Turret motor position", TurretMotorPos);

    Logger.recordOutput(logRoot + "Hood Position", HoodPos);

    Logger.recordOutput(logRoot + "Bottom Side Trigger", onBottomHalf());

    Logger.recordOutput(logRoot + "Top Side Trigger", onTopHalf());
    Logger.recordOutput(logRoot + "Team Side Trigger", onTeamSide());

    Logger.recordOutput(logRoot + "Distance", distance);

    // {
    // String calcLogRoot = logRoot + "ClosestAlignment/";
    // Logger.recordOutput(
    // calcLogRoot + "Type",
    // getClosestAlignmentTracker().getClass().getSimpleName());
    // }

    {
      hubTracker.update();

      String calcLogRoot = logRoot + "Hub/";
      Logger.recordOutput(calcLogRoot + "Closest tag", FieldUtils.getClosestHub().tag);
      Logger.recordOutput(
          calcLogRoot + "Target Angle Deg", BobotState.getRotationtoClosestHub().getDegrees());
    }
    {
      TestTarget.update();
      String calcLogRoot = logRoot + "Target/";
      Logger.recordOutput(calcLogRoot + "Closest tag", FieldUtils.getClosestHub().tag);
      Logger.recordOutput(
          calcLogRoot + "Target Angle Deg", BobotState.getRotationtoClosestHub().getDegrees());
    }
  }

  @Override
  public void simulationPeriodic() {}
}
