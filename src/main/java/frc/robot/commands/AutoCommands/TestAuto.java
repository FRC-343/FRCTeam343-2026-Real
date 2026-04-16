package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state2.BobotState;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePiviot.IntakePiviot;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.LowerShooter.LowerShooter;
import frc.robot.subsystems.UpperShooter.UpperShooter;

public class TestAuto {
  Path trenchLeave = new Path("Test");

      public Command LeftShoot(Intake intake, LowerShooter lShoot, UpperShooter uShoot, Kicker
  kicker, IntakePiviot iPiviot){
        return Commands.sequence( iPiviot.setAngle(0), Commands.parallel(intake.setPercentOutputThenStopCommand(-.5),
            Commands.parallel(BobotState.getBuilder().build(trenchLeave))
        ));
      }
}
