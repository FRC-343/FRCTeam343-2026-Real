package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class DriveCommandConstants {
  public static ProfiledPIDController makeAngleController() {
    ProfiledPIDController angleController =
        new ProfiledPIDController(10.0, 0.0, 8, new TrapezoidProfile.Constraints(4.0, 10.0));

    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(1.5));

    return angleController;
  }

  public static PIDController makeTranslationController() {
    PIDController translationController = new PIDController(3.5, 0.0, 0);
    translationController.setTolerance(Units.inchesToMeters(1));

    return translationController;
  }
}
