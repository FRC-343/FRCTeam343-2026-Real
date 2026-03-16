// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.bobot_state2.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Spindexer.Spindexer;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision2.Vision;
import frc.robot.util.CommandCustomController;
import frc.robot.util.ShooterHelper.HoodAimTest;
import frc.robot.util.ShooterHelper.ShooterSpeed;
import frc.robot.util.ShooterHelper.TimeOfFlightTest;
import frc.robot.util.ShooterHelper.TurretCalc;
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

  // private final Shooter shooter;

  private final Turret turret;

  private final Spindexer spindexer;

  private final Kicker kicker;

  private final BobotState bobot;

  private final Intake intake;

  private final Shooter shooter;

  private final Hood hood;

  // Controller
  private final CommandCustomController controller = new CommandCustomController(0);
  private final CommandCustomController controller2 = new CommandCustomController(1);

  // private final DriverAutomationFactory m_Automation;

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

        // m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        // shooter = new Shooter();
        turret = new Turret();
        spindexer = new Spindexer();
        kicker = new Kicker();
        intake = new Intake();
        hood = new Hood();
        shooter = new Shooter();
        bobot = new BobotState();
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
        // m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        // shooter = new Shooter();
        turret = new Turret();
        spindexer = new Spindexer();
        kicker = new Kicker();
        intake = new Intake();
        hood = new Hood();
        shooter = new Shooter();
        bobot = new BobotState();
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

        // m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        // shooter = new Shooter();
        turret = new Turret();
        spindexer = new Spindexer();
        kicker = new Kicker();
        intake = new Intake();
        hood = new Hood();
        shooter = new Shooter();
        bobot = new BobotState();
        new Vision();
        break;
    }
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
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Intake", intake.runForTime(.5, 5));
    NamedCommands.registerCommand("Shooter set speed", shooter.runForTime(40, 10));
    NamedCommands.registerCommand("Spindexer", spindexer.runForTime(-35, 10));
    NamedCommands.registerCommand("Kicker", kicker.runForTime(20, 10));
    NamedCommands.registerCommand("Hood", hood.setHoodPosition2());
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

    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> BobotState.getRotationToTarget()));
    controller.rightTrigger().whileTrue(intake.setPercentOutputThenStopCommand(.5));
    controller.leftTrigger().whileTrue(intake.setPercentOutputThenStopCommand(-.5));
    controller.leftBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));
  }

  private void configureOpButtons() {
    controller2
        .a()
        .whileTrue(shooter.setVelocityThenStopCommand().alongWith(hood.setHoodPosition()));
    controller2
        .leftBumper()
        .whileTrue(
            spindexer
                .setVelocityThenStopCommand(-35)
                .alongWith(kicker.setVelocityThenStopCommand(20)));

    controller2
        .y()
        .whileTrue(shooter.setVelocityThenStopCommand2(40).alongWith(hood.setHoodPosition2()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void ShootCalcs() {

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            BobotState.getRoboSpeed(), BobotState.getGlobalPose().getRotation());

    Translation2d robotVelocity =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    Translation2d target = BobotState.getTurretTarget().getTranslation();

    Translation2d robotPos = BobotState.getGlobalPose().getTranslation();

    double distance = robotPos.getDistance(target);

    BobotState.updateDistance(distance);

    double hood = HoodAimTest.chooseHoodAngle(distance);

    double velocity = HoodAimTest.requiredVelocity(distance, Units.inchesToMeters(55), hood);

    double horizontalVelocity = velocity * Math.cos(Math.toRadians(hood));

    double time =
        TimeOfFlightTest.solveTime(
            robotPos, robotVelocity, target, new Translation2d(), horizontalVelocity);
    Translation2d leadTarget;
    if (!Double.isNaN(time)) {

      leadTarget = target.plus(robotVelocity.times(time));

    } else {
      leadTarget = target;
    }
    Rotation2d robotHeading = BobotState.getGlobalPose().getRotation();

    Translation2d toTarget = leadTarget.minus(robotPos);

    // field angle to target
    Rotation2d targetAngle = new Rotation2d(toTarget.getX(), toTarget.getY());
    BobotState.updateBotAngle(targetAngle);

    // turret should point here relative to robot
    double turretSetpointRadians =
        MathUtil.angleModulus(targetAngle.minus(robotHeading).getRadians());

    double turretRotations = TurretCalc.turretRadiansToMotorRotations(turretSetpointRadians);

    // BobotState.updateOptiTurretYaw(turretRotations);
    BobotState.updateHoodAngle(hood);
  }

  public void Automation() {
    BobotState.updateTurretTarget(BobotState.targetLocation());
    BobotState.updateWantedShooterRPS(ShooterSpeed.ShooterRPS(BobotState.getDistance()));
  }
}
