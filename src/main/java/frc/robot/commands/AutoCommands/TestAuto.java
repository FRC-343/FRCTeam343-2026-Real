package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state2.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePiviot.IntakePiviot;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.LowerShooter.LowerShooter;
import frc.robot.subsystems.UpperShooter.UpperShooter;
import frc.robot.subsystems.drive.Drive;

public class TestAuto {
  static Path trenchLeave = new Path("LeftTrenchLeave");

  public static Command LeftShoot(
      Intake intake,
      LowerShooter lShoot,
      UpperShooter uShoot,
      Kicker kicker,
      IntakePiviot iPiviot,
      Drive drive) {
    return Commands.sequence(
        iPiviot.setAngle(0).withTimeout(.2),
        Commands.parallel(
            intake.setPercentOutputThenStopCommand(-.5),
            Commands.sequence(
                drive.pathBuilder().build(trenchLeave),
                DriveCommands.pointAtAngle(
                        drive, () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle() + 180))
                    .withTimeout(4))));
  }
}
