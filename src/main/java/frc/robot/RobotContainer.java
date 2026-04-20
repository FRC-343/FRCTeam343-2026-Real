// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.bobot_state2.BobotState;
import frc.robot.commands.BlineAuto.BlineAutos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePiviot.IntakePiviot;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.LowerShooter.LowerShooter;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.UpperShooter.UpperShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision2.Vision;
import frc.robot.util.CommandCustomController;
import frc.robot.util.TurretStuff.TurretUtil;
import frc.robot.util.TurretStuff.TurretUtil.TargetType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final Spindexer spindexer;

  private final Kicker kicker;

  private final BobotState bobot;

  private final Intake intake;

  private final UpperShooter upperShooter;

  private final LowerShooter lowerShooter;

  private final Hood hood;

  private final IntakePiviot iPiviot;

  // Controller
  private final CommandCustomController controller = new CommandCustomController(0);
  private final CommandCustomController controller2 = new CommandCustomController(1);

  private final CommandCustomController testController = new CommandCustomController(4);

  private final DriverAutomationFactory m_Automation;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        spindexer = new Spindexer();
        kicker = new Kicker();
        intake = new Intake();
        hood = new Hood();
        upperShooter = new UpperShooter();
        bobot = new BobotState();
        lowerShooter = new LowerShooter();
        iPiviot = new IntakePiviot();
        new Vision();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        // shooter = new Shooter();
        spindexer = new Spindexer();
        kicker = new Kicker();
        intake = new Intake();
        hood = new Hood();
        upperShooter = new UpperShooter();
        bobot = new BobotState();
        lowerShooter = new LowerShooter();
        iPiviot = new IntakePiviot();
        new Vision();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        // shooter = new Shooter();
        spindexer = new Spindexer();
        kicker = new Kicker();
        intake = new Intake();
        hood = new Hood();
        upperShooter = new UpperShooter();
        bobot = new BobotState();
        lowerShooter = new LowerShooter();
        iPiviot = new IntakePiviot();
        new Vision();
        break;
    }
    SmartDashboard.putNumber("Wait time", 0);

    configureNamedCommands();
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureOpButtons();
    configureTestButtons();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Intake for time", intake.runForTime(.5, 5));
    NamedCommands.registerCommand("Intake no stop", intake.setPercentOutputThenStopCommand(-.5));
    NamedCommands.registerCommand(
        "Shooter set speed", upperShooter.setVelocityThenStopCommand().withTimeout(4));
    NamedCommands.registerCommand("Shooter no stop", upperShooter.setVelocityThenStopCommand());
    NamedCommands.registerCommand(
        "Spindexer", spindexer.setVelocityThenStopCommand(-9).withTimeout(3));
    NamedCommands.registerCommand("Kicker", kicker.setVelocityThenStopCommand(20).withTimeout(3));
    NamedCommands.registerCommand("Hood", hood.setHoodPosition2().withTimeout(5));
    NamedCommands.registerCommand("Hood down", hood.setHoodPosition(2).withTimeout(.5));

    NamedCommands.registerCommand(
        "AutoAim",
        DriveCommands.pointAtAngle(
                drive, () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle()))
            .withTimeout(4));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.rightTrigger().whileTrue(intake.setPercentOutputThenStopCommand(-.5));
    controller
        .b()
        .whileTrue(
            intake
                .setPercentOutputThenStopCommand(-.5)
                .alongWith(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));
    controller
        .y()
        .whileTrue(
            DriveCommands.pointAtAngle(
                drive, () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle())));
    controller.leftTrigger().whileTrue(intake.setPercentOutputThenStopCommand(.5));
    controller.leftBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.rightBumper().whileTrue(intake.jiggle(.5, .2));

    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngleForShoot(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.fromDegrees(BobotState.getSolutionAngle())));

    controller
        .x()
        .whileTrue(
            intake
                .setPercentOutputThenStopCommand(.5)
                .alongWith(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> new Rotation2d(controller.getLeftY(), controller.getLeftX()))));
  }

  private void configureOpButtons() {

    controller2
        .a()
        .whileTrue(
            upperShooter
                .setVelocityThenStopCommand()
                .alongWith(lowerShooter.setVelocityThenStopCommand())
                .alongWith(hood.setHoodPosition2()));
    controller2
        .leftBumper()
        .whileTrue(
            spindexer
                .setVelocityThenStopCommand(-5)
                .alongWith(kicker.setVelocityThenStopCommand(14)));
    controller2
        .leftTrigger()
        .whileTrue(
            kicker
                .setVelocityThenStopCommand(55)
                .alongWith(spindexer.setVelocityThenStopCommand(-25)));

    controller2
        .rightBumper()
        .whileTrue(
            spindexer
                .setVelocityThenStopCommand(5)
                .alongWith(kicker.setVelocityThenStopCommand(-55)));
    controller2
        .rightTrigger()
        .whileTrue(
            spindexer
                .setVelocityThenStopCommand(-5)
                .alongWith(kicker.setVelocityThenStopCommand(18)));

    controller2
        .y()
        .whileTrue(
            upperShooter
                .setVelocityThenStopCommand()
                .alongWith(lowerShooter.setVelocityThenStopCommand())
                .alongWith(hood.setHoodPosition2()))
        .whileFalse(hood.setHoodPosition(2));
  }

  private void configureTestButtons() {
    testController
        .rightBumper()
        .whileTrue(
            upperShooter
                .setVelocityThenStopCommand2(-32)
                .alongWith(lowerShooter.setVelocityThenStopCommand2(32)));
    testController.leftBumper().whileTrue(kicker.setVelocityThenStopCommand(58));

    // testController.rightTrigger().whileTrue(intake.setPercentOutputThenStopCommand(.5));
    // testController.leftTrigger().whileTrue(intake.setPercentOutputThenStopCommand(-.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void TurretMath() {
    TargetType target = BobotState.targetType();
    Pose2d robotPose = BobotState.getGlobalPose();
    ChassisSpeeds speeds = BobotState.getRoboSpeed();
    TurretUtil.ShotSolution solution =
        TurretUtil.computeLeadShotSolution(
            robotPose, speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, target);

    if (solution.isValid) {
      BobotState.updateOptiTurretYaw(
          TurretUtil.degreesToMotorRotations(solution.turretAngleDegrees));
      BobotState.updateSolutionDegAngle(
          solution.turretAngleDegrees + BobotState.getGlobalPose().getRotation().getDegrees());
      BobotState.updateWantedShooterRPS(solution.shooterSpeedRPS);
      BobotState.updateHoodAngle(solution.trajectoryAngleDegrees);
      BobotState.updateDistance(solution.distanceMeters);
    }
  }

  public void Automation() {
    BobotState.updateTurretTarget(BobotState.targetLocation());
    BobotState.updatewaitTest(SmartDashboard.getNumber("Wait time", 0));
  }

  public void UpdatingAutos() {

    autoChooser.addOption(
        "RightShoot",
        BlineAutos.RightShoot(
            intake, lowerShooter, upperShooter, kicker, iPiviot, drive, BobotState.getWaitTest()));
  }
}
