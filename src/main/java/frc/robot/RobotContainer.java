// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.TargetTest.DashboardTarget;
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
import frc.robot.util.ShooterHelper.HoodAim;
import frc.robot.util.ShooterHelper.TimeOfFlight;
import frc.robot.util.ShooterHelper.TurretCRT2;
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

  //   private final Shooter shooter;

  private final Turret turret;

  private final Spindexer spindexer;

  private final Kicker kicker;

  private final BobotState test;

  private final Intake intake;

  private final DashboardTarget test2;

  private final Shooter shooter;

  private final Hood hood;

  // Controller
  private final CommandCustomController controller = new CommandCustomController(0);
  private final CommandCustomController controller2 = new CommandCustomController(1);

  //   private final DriverAutomationFactory m_Automation;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    test = new BobotState();
    new Vision();
    test2 = new DashboardTarget();
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
        break;
    }

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

    // controller.rightBumper().whileTrue(turret.setTurretPosition2());

    controller.rightTrigger().whileTrue(intake.setPercentOutputThenStopCommand(.45));
    controller.leftTrigger().whileTrue(intake.setPercentOutputThenStopCommand(-.45));
  }

  private void configureOpButtons() {
    controller2
        .a()
        .whileTrue(shooter.setVelocityThenStopCommand(20).alongWith(hood.setHoodPosition()));
    controller2
        .leftBumper()
        .whileTrue(
            spindexer
                .setVelocityThenStopCommand(-25)
                .alongWith(kicker.setVelocityThenStopCommand(40)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void TurretCalcs() {

    BobotState.updateTurretTarget(test2.getTarget());
    BobotState.updateOptiTurretYaw(
        TurretCRT2.turretRadiansToMotorRotations(
            TurretCRT2.calculateTurretSetpointRadians(
                BobotState.getTurretTarget(),
                BobotState.getGlobalPose(),
                Rotation2d.fromRotations(BobotState.getTurretPosi2()))));
  }

  public void HoodCalcs() {

    /* Field Speed converts robot speed to field speeds for easier math */
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            BobotState.getRoboSpeed(), BobotState.getGlobalPose().getRotation());

    /* Gets the robot velocity using the converted field speeds */
    Translation2d robotVelocityXY =
        new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    // /* Gets the shooter position on the robot */
    // Translation2d shooterXY =
    //     BobotState.getGlobalPose()
    //         .transformBy(
    //             new Transform2d(
    //                 Units.inchesToMeters(-2.25), Units.inchesToMeters(-4.8125), new
    // Rotation2d()))
    //         .getTranslation();

    /* Gets the targerts position on the field
     *
     */

    Translation2d targetXY = test2.getTarget();

    BobotState.updateTurretTarget(
        targetXY); // This mainly used for sim to show the position the turret/hood is targeting.

    /* Gets the fuel exit velocity this is used for the hood calculations */
    double shooterExitVelocity =
        BobotState.getShooterRPM() * Constants.ShooterConstants.WheelCir * .3;

    /* Call for the calculation that gets an estimated time that the fuel will be in the air from
    any give position/speed */
    BobotState.updateToF(
        TimeOfFlight.solveTime(
            BobotState.getGlobalPose().getTranslation(),
            robotVelocityXY,
            targetXY,
            new Translation2d(),
            shooterExitVelocity));

    double time = BobotState.getToF(); // gives us an easier call for the TOF

    if (!Double.isNaN(time)) { // If time is a real number do the calculations

      Translation2d intercept =
          targetXY.plus(
              robotVelocityXY.times(-time)); // Gets the positon that the robot would hit the

      double distance =
          intercept.getDistance(
              BobotState.getGlobalPose()
                  .getTranslation()); // Gets the distance from the shooter to the intercept
      // position.

      double hood = HoodAim.calculateHoodAngle(distance, 55, shooterExitVelocity); // Gets the hood

      /* updates our call for the hood and turret */
      BobotState.updateHoodAngle(hood);
    }
  }
}
