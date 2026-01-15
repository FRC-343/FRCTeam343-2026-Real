package frc.robot;

// import frc.robot.commands.PositionWithCoralStation;
// import frc.robot.commands.PositionWithReef;
import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.vision.OffsetTags;
import frc.robot.util.CommandCustomController;

public class DriverAutomationFactory {
  private final CommandCustomController driverController;
  private final CommandCustomController operatorController;

  private final Drive drive;

  public DriverAutomationFactory(
      CommandCustomController driverController,
      CommandCustomController operatorController,
      Drive drive) {
    this.driverController = driverController;
    this.operatorController = operatorController;
    this.drive = drive;
  }
}
