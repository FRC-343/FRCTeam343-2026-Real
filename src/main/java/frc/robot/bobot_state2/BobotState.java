package frc.robot.bobot_state2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.bobot_state2.varc.HubTagTracker;
import frc.robot.bobot_state2.varc.TargetAngleTracker;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldUtils;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.vision2.PoseObservation;
import frc.robot.util.TurretStuff.TurretUtil;
import frc.robot.util.TurretStuff.TurretUtil.TargetType;
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

  private static String leftPathOne; // The Path that happens after a Left Leave Path

  private static String leftPathTwo; // The Path that happens after Left Path One

  private static String leftPathThree; // The Path that happens after Left Path two

  private static String rightPathOne; // The Path that happens after a right Leave Path

  private static String rightPathTwo; // The Path that happens after right Path One

  private static String rightPathThree; // The Path that happens after right Path two

  private static double startWait;

  private static double bumpWait;

  private static double leaveWait;

  private static double bumpTwoWait;

  private static FollowPath.Builder BuilderTest;

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

  private static boolean FlipCheck;

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

  private static double hoodTest;

  private static double shooterTest;

  private static Pose2d turretPose;

  private static boolean slowdown;

  private static double solutionDegAngle;

  public static void updateFlipCheck(boolean FlipCheck) {
    BobotState.FlipCheck = FlipCheck;
  }

  public static void updateLeftPathOne(String LPOne) {
    BobotState.leftPathOne = LPOne;
  }

  public static void updateLeftPathTwo(String LPTwo) {
    BobotState.leftPathTwo = LPTwo;
  }

  public static void updateLeftPathThree(String LPThree) {
    BobotState.leftPathThree = LPThree;
  }

  public static void updateRightPathOne(String RPOne) {
    BobotState.rightPathOne = RPOne;
  }

  public static void updateRightPathTwo(String RPTwo) {
    BobotState.rightPathTwo = RPTwo;
  }

  public static void updateRightPathThree(String RPThree) {
    BobotState.rightPathThree = RPThree;
  }

  public static void updateStartWait(double startWait) {
    BobotState.startWait = startWait;
  }

  public static void updateBumpWait(double bumpWait) {
    BobotState.bumpWait = bumpWait;
  }

  public static void updateLeaveWait(double leaveWait) {
    BobotState.leaveWait = leaveWait;
  }

  public static void updateBumpTwoWait(double bumpTwoWait) {
    BobotState.bumpTwoWait = bumpTwoWait;
  }

  public static void setPathBuilder(FollowPath.Builder builder) {
    BobotState.BuilderTest = builder;
  }

  public static void updateSolutionDegAngle(double angle) {
    BobotState.solutionDegAngle = angle;
  }

  public static void updateSlowdown(boolean slow) {
    BobotState.slowdown = slow;
  }

  public static void updateTurretPose(Pose2d turret) {
    BobotState.turretPose = turret;
  }

  public static void updateHoodTest(double value) {
    BobotState.hoodTest = value;
  }

  public static void updateShooterTest(double value) {
    BobotState.shooterTest = value;
  }

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

  public static boolean getSlowdown() {
    return BobotState.slowdown;
  }

  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  public static boolean getFlipCheck() {
    return BobotState.FlipCheck;
  }

  /*
   *
   * Section we are adding get methods to
   *
   */

  public static String getLeftPathOne() {
    return BobotState.leftPathOne;
  }

  public static String getLeftPathTwo() {
    return BobotState.leftPathTwo;
  }

  public static String getLeftPathThree() {
    return BobotState.leftPathThree;
  }

  public static String getRightPathOne() {
    return BobotState.rightPathOne;
  }

  public static String getRightPathTwo() {
    return BobotState.rightPathTwo;
  }

  public static String getRightPathThree() {
    return BobotState.rightPathThree;
  }

  public static double getStartWait() {
    return BobotState.startWait;
  }

  public static double getBumpWait() {
    return BobotState.bumpWait;
  }

  public static double getLeaveWait() {
    return BobotState.leaveWait;
  }

  public static double getBumpTwoWait() {
    return BobotState.bumpTwoWait;
  }

  public static double getSolutionAngle() {
    return BobotState.solutionDegAngle;
  }

  public static Pose2d getTurretPose() {
    return BobotState.turretPose;
  }

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

  public static Pose2d getTurretTarget() {
    return BobotState.TurretTarget;
  }

  public static double getHoodPos() {
    return BobotState.HoodPos;
  }

  public static double getWantedHood() {
    return BobotState.HoodCalc;
  }

  public static Trigger slowTrigger() {
    return new Trigger(() -> (BobotState.TurretPos2 < 1.0 && BobotState.TurretPos2 > -1.0));
  }

  public static Pose2d targetLocation() {
    return (onTeamSide().getAsBoolean()
        ? FieldUtils.getHub()
        : onTopHalf().getAsBoolean()
            ? FieldUtils.getLeftTarget()
            : onBottomHalf().getAsBoolean() ? FieldUtils.getRightTarget() : FieldUtils.getHub());
  }

  public static TargetType targetType() {
    return (onTeamSide().getAsBoolean()
        ? TurretUtil.TargetType.HUB
        : onTopHalf().getAsBoolean()
            ? TurretUtil.TargetType.LEFT_PASS
            : onBottomHalf().getAsBoolean()
                ? TurretUtil.TargetType.RIGHT_PASS
                : TurretUtil.TargetType.HUB);
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

    Logger.recordOutput(logRoot + "Turret pose", turretPose);

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

    Logger.recordOutput(logRoot + "slow down trigger", slowTrigger().getAsBoolean());

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

    Logger.recordOutput(logRoot + "Target Type", targetType().toString());

    Logger.recordOutput(logRoot + "Hood Increment value", hoodTest);

    Logger.recordOutput(logRoot + "ShooterRPS Increment value", shooterTest);

    Logger.recordOutput(logRoot + "Slowdown", slowdown);

    Logger.recordOutput(logRoot + "Solution Angle", solutionDegAngle);

    Logger.recordOutput(logRoot + "Start Wait", startWait);

    Logger.recordOutput(logRoot + "Bump Wait", bumpWait);

    Logger.recordOutput(logRoot + "Leave Wait", leaveWait);

    Logger.recordOutput(logRoot + "Bump Two Wait", bumpTwoWait);

    Logger.recordOutput(logRoot + "Left Path One", leftPathOne);

    Logger.recordOutput(logRoot + "Left Path Two", leftPathTwo);

    Logger.recordOutput(logRoot + "Left Path Three", leftPathThree);

    Logger.recordOutput(logRoot + "Right Path One", rightPathOne);

    Logger.recordOutput(logRoot + "Right Path Two", rightPathTwo);

    Logger.recordOutput(logRoot + "Right Path Three", rightPathThree);

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
  }

  @Override
  public void simulationPeriodic() {}
}
