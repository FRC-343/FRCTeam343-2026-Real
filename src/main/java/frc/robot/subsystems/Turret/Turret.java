package frc.robot.subsystems.Turret;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.bobot_state2.BobotState;
import frc.robot.util.ShooterHelper.TurretCRT;
import org.littletonrobotics.junction.Logger;

/*
 * self explanatory
 * Do note that out Visulizer is not currently working as of 2/6/2025
 */

public class Turret extends SubsystemBase {
  private final TurretMotorIO io;

  boolean test = false;

  private final TurretMotorIOInputsAutoLogged inputs = new TurretMotorIOInputsAutoLogged();

  private final PIDController pidController =
      new PIDController(
          0.5, // Replace with actual PID values when on the bot
          0, 0);

  private final TurretVisualizer measuredVisualizer =
      new TurretVisualizer("Measured", Color.kBlack);
  private final TurretVisualizer setpointVisualizer =
      new TurretVisualizer("Setpoint", Color.kGreen);

  private double setpointInches = 0.0;

  public Turret() {
    switch (Constants.currentMode) {
      case REAL:
        io = new TurretMotorTalonFX(13, 17, 18);

        break;
      case SIM:
        io = new TurretMotorSim(DCMotor.getKrakenX60(1), 3, 1, new PIDConstants(1, 0, 0));

        break;
      case REPLAY:
      default:
        io = new TurretMotorIO() {};
        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);

    Logger.processInputs("Turret", this.inputs);

    if (DriverStation.isDisabled()) {
      this.setSetpoint(0.0);
      this.io.stop();
    }

    Logger.recordOutput("Turret/SetpointInches", setpointInches);

    // Log Mechanisms
    // measuredVisualizer.update(this.inputs.masterPositionRad);
    // setpointVisualizer.update(this.setpointInches);
    // // I'm not quite sure how this works, it is semi working in sim.

    // BobotState.updateTurretPos(this.inputs.masterPositionRad);

    BobotState.updateR13AbsPos(this.inputs.r13Abspos);
    BobotState.updateR17AbsPos(this.inputs.r17Abspos);

    double r17 = BobotState.getR13AbsPos() * 17.0; // 13T gear → mod 17
    double r13 = BobotState.getR17AbsPos() * 13.0; // 17T gear → mod 13

    // Reconstruct X in [0,221)
    double X = TurretCRT.reconstruct(r17, r13);

    // Convert to turret rotations
    double turretRot = X / 221.0;

    // Convert to radians
    double turretRad = TurretCRT.turretRotToRadians(turretRot);

    // Apply zero offset (store once at calibration)
    // turretRad -= TURRET_ZERO_OFFSET_RAD;

    BobotState.updateTurretPos(turretRad);
  }

  // These needs to be reorganized

  private void setSetpoint(double setpoint) {
    setpointInches = MathUtil.clamp(setpoint, 0, 56); // not real value
    this.io.setSetpoint(this.setpointInches);
  }

  public Command setSetpointCommand(double positionInches) {
    return new InstantCommand(() -> this.setSetpoint(positionInches));
  }

  public Command stopCommand() {

    return new InstantCommand(this.io::stop, this);
  }

  public void setVoltage(double voltage) {
    this.io.setTurretVelocity(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public Command setVolatageCommand(double voltage) {
    return new RunCommand(() -> this.io.setTurretVelocity(voltage), this);
  }

  public Command resetEncoder() {
    return new InstantCommand(this.io::resetEncoder, this);
  }

  public Command setPercentOutputCommand(double velocityRotPerSecond) {
    setpointInches = velocityRotPerSecond * 1000;
    return new RunCommand(() -> this.io.setPercentOutput(velocityRotPerSecond), this);
  }

  /*
   * Adding new commands down here to ease readability,
   * at some point the above commands will be reorganized.
   * Triggers might also be separated at a later date, potentially added to
   * BobotState
   */
  public Command setTurretPosition() {
    return new RunCommand(() -> this.io.setTurretPosition(-BobotState.getMotorTarget()));
  }
}
