package frc.robot.commands.BlineAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePiviot.IntakePiviot;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.LowerShooter.LowerShooter;
import frc.robot.subsystems.UpperShooter.UpperShooter;
import frc.robot.subsystems.drive.Drive;

public class BlineAutos {

  static Path RightBumpCrossing = new Path("RightBumpCrossing");

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
      String RightPathOne) {
    Path RPO = new Path(RightPathOne);
    Path RightLeave = new Path("RightLeave");

    if (RightPathOne == "RightToBump") {
      return Commands.sequence(
          Commands.waitSeconds(startWait),
          iPiviot.setAngle(0).withTimeout(.2),
          Commands.parallel(
              intake.setPercentOutputThenStopCommand(-.5),
              Commands.sequence(
                  drive.pathBuilder().build(RightLeave),
                  drive.pathBuilder().build(RPO),
                  Commands.waitSeconds(bumpOneWait),
                  drive.pathBuilder().build(RightBumpCrossing))));
    } else if (RightPathOne == "RightToOpHub") {
      return Commands.sequence(
          Commands.waitSeconds(startWait),
          iPiviot.setAngle(0).withTimeout(.2),
          Commands.parallel(
              intake.setPercentOutputThenStopCommand(-.5),
              Commands.sequence(
                  drive.pathBuilder().build(RightLeave),
                  drive.pathBuilder().build(RPO),
                  Commands.waitSeconds(bumpOneWait))));

    } else {
      return null;
    }
  }
}
