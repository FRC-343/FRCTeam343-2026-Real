package frc.robot.commands.BlineAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.bobot_state2.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePiviot.IntakePiviot;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.LowerShooter.LowerShooter;
import frc.robot.subsystems.UpperShooter.UpperShooter;
import frc.robot.subsystems.drive.Drive;

public class BlineAutos {

  static Path RightBumpCrossing = new Path("RightBumpCrossing");
  static Path RightToTrench = new Path("RightToTrench");
  static Path RightLeavePTwo = new Path("RightLeavePTwo");

  static Path SanityCheck = new Path("SanityPath");

  static double wait = SmartDashboard.getNumber("Wait time", 0);

  public static Command RightShoot(
      Intake intake,
      LowerShooter lShoot,
      UpperShooter uShoot,
      Kicker kicker,
      IntakePiviot iPiviot,
      Drive drive,
      double startWait,
      double bumpOneWait,
      double LeaveWait,
      double bumpTwoWait,
      String RightPathOne,
      String RightPathTwo) {
    Path RPO = new Path(RightPathOne);
    Path RPT = new Path(RightPathTwo);
    Path RightLeave = new Path("RightLeave");

    if (RightPathOne == "RightToBump"
        && (RightPathTwo == "RightScore2Close" || RightPathTwo == "RightScore2Far")) {
      return Commands.sequence(
          Commands.waitSeconds(startWait),
          iPiviot.setAngle(IntakeConstants.INTAKEDOWN).withTimeout(.2),
          Commands.parallel(
              intake.setPercentOutputThenStopCommand(-.5),
              Commands.sequence(
                  drive.pathBuilder().withPoseReset(drive::setPose).build(RightLeave),
                  drive.pathBuilder().withPoseReset(p -> {}).build(RPO),
                  Commands.waitSeconds(bumpOneWait),
                  drive.pathBuilder().build(RightBumpCrossing),
                  Commands.parallel(
                      DriveCommands.pointAtAngle(
                              drive,
                              () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle() + 180))
                          .withTimeout(3.5),
                      lShoot.setVelocityThenStopCommand().withTimeout(3.5),
                      uShoot.setVelocityThenStopCommand().withTimeout(3.5),
                      Commands.sequence(
                          Commands.waitSeconds(1),
                          kicker.setVelocityThenStopCommand(18).withTimeout(2.5))),
                  drive.pathBuilder().build(RightToTrench),
                  Commands.waitSeconds(LeaveWait),
                  drive.pathBuilder().build(RightLeavePTwo),
                  drive.pathBuilder().build(RPT),
                  Commands.waitSeconds(bumpTwoWait),
                  drive.pathBuilder().build(RightBumpCrossing),
                  Commands.parallel(
                      DriveCommands.pointAtAngle(
                              drive, () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle() + 180))
                          .withTimeout(4),
                      lShoot.setVelocityThenStopCommand().withTimeout(4),
                      uShoot.setVelocityThenStopCommand().withTimeout(4),
                      Commands.sequence(
                          Commands.waitSeconds(1),
                          kicker.setVelocityThenStopCommand(18).withTimeout(3))))));
    } else if (RightPathOne == "RightToBump" && RightPathTwo == "RightToHPS") {
      return Commands.sequence(
          Commands.waitSeconds(startWait),
          iPiviot.setAngle(-0.25).withTimeout(.2),
          Commands.parallel(
              intake.setPercentOutputThenStopCommand(-.5),
              Commands.sequence(
                  drive.pathBuilder().withPoseReset(drive::setPose).build(RightLeave),
                  drive.pathBuilder().withPoseReset(p -> {}).build(RPO),
                  Commands.waitSeconds(bumpOneWait),
                  drive.pathBuilder().build(RightBumpCrossing),
                  Commands.parallel(
                      DriveCommands.pointAtAngle(
                              drive,
                              () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle() + 180))
                          .withTimeout(3.5),
                      lShoot.setVelocityThenStopCommand().withTimeout(3.5),
                      uShoot.setVelocityThenStopCommand().withTimeout(3.5),
                      Commands.sequence(
                          Commands.waitSeconds(1),
                          kicker.setVelocityThenStopCommand(18).withTimeout(2.5))),
                  drive.pathBuilder().build(RPT))));
    } else {
      return null;
    }
  }

  public static Command SanityCheck(Drive drive) {
    return Commands.sequence(
        Commands.waitSeconds(1),
        drive.pathBuilder().withPoseReset(drive::setPose).build(SanityCheck));
  }

  public static Command trenchShoot(
      Intake intake,
      LowerShooter lShoot,
      UpperShooter uShoot,
      Kicker kicker,
      IntakePiviot iPiviot,
      Drive drive,
      double startWait,
      double bumpOneWait,
      double LeaveWait,
      double bumpTwoWait,
      String RightPathOne,
      String RightPathTwo) {
    Path RPO = new Path(RightPathOne);
    Path RPT = new Path(RightPathTwo);
    Path RightLeave = new Path("RightLeave");

    if (BobotState.getFlipCheck()) {
      RPO.mirror();
      RPT.mirror();
      RightLeave.mirror();
      RightBumpCrossing.mirror();
      RightToTrench.mirror();
      RightLeavePTwo.mirror();
    }

    if (RightPathOne == "RightToBump"
        && (RightPathTwo == "RightScore2Close" || RightPathTwo == "RightScore2Far")) {
      return Commands.sequence(
          Commands.waitSeconds(startWait),
          iPiviot.setAngle(IntakeConstants.INTAKEDOWN).withTimeout(.2),
          Commands.parallel(
              intake.setPercentOutputThenStopCommand(-.5),
              Commands.sequence(
                  drive.pathBuilder().withPoseReset(drive::setPose).build(RightLeave),
                  drive.pathBuilder().withPoseReset(p -> {}).build(RPO),
                  Commands.waitSeconds(bumpOneWait),
                  drive.pathBuilder().build(RightBumpCrossing),
                  Commands.parallel(
                      DriveCommands.pointAtAngle(
                              drive,
                              () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle() + 180))
                          .withTimeout(3.5),
                      lShoot.setVelocityThenStopCommand().withTimeout(3.5),
                      uShoot.setVelocityThenStopCommand().withTimeout(3.5),
                      Commands.sequence(
                          Commands.waitSeconds(1),
                          kicker.setVelocityThenStopCommand(18).withTimeout(2.5))),
                  drive.pathBuilder().build(RightToTrench),
                  Commands.waitSeconds(LeaveWait),
                  drive.pathBuilder().build(RightLeavePTwo),
                  drive.pathBuilder().build(RPT),
                  Commands.waitSeconds(bumpTwoWait),
                  drive.pathBuilder().build(RightBumpCrossing),
                  Commands.parallel(
                      DriveCommands.pointAtAngle(
                              drive, () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle() + 180))
                          .withTimeout(4),
                      lShoot.setVelocityThenStopCommand().withTimeout(4),
                      uShoot.setVelocityThenStopCommand().withTimeout(4),
                      Commands.sequence(
                          Commands.waitSeconds(1),
                          kicker.setVelocityThenStopCommand(18).withTimeout(3))))));
    } else if (RightPathOne == "RightToBump" && RightPathTwo == "RightToHPS") {
      return Commands.sequence(
          Commands.waitSeconds(startWait),
          iPiviot.setAngle(-0.25).withTimeout(.2),
          Commands.parallel(
              intake.setPercentOutputThenStopCommand(-.5),
              Commands.sequence(
                  drive.pathBuilder().withPoseReset(drive::setPose).build(RightLeave),
                  drive.pathBuilder().withPoseReset(p -> {}).build(RPO),
                  Commands.waitSeconds(bumpOneWait),
                  drive.pathBuilder().build(RightBumpCrossing),
                  Commands.parallel(
                      DriveCommands.pointAtAngle(
                              drive,
                              () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle() + 180))
                          .withTimeout(3.5),
                      lShoot.setVelocityThenStopCommand().withTimeout(3.5),
                      uShoot.setVelocityThenStopCommand().withTimeout(3.5),
                      Commands.sequence(
                          Commands.waitSeconds(1),
                          kicker.setVelocityThenStopCommand(18).withTimeout(2.5))),
                  drive.pathBuilder().build(RPT))));
    } else {
      return null;
    }
  }
}
