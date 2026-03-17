package frc.robot.subsystems.Hood;

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
import org.littletonrobotics.junction.Logger;

/*
 * self explanatory
 * Do note that out Visulizer is not currently working as of 2/6/2025
 */

public class Hood extends SubsystemBase {
  private final HoodMotorIO io;

  boolean test = false;

  private final HoodMotorIOInputsAutoLogged inputs = new HoodMotorIOInputsAutoLogged();

  private final PIDController pidController =
      new PIDController(
          0.5, // Replace with actual PID values when on the bot
          0, 0);

  private final HoodVisualizer measuredVisualizer = new HoodVisualizer("Measured", Color.kBlack);
  private final HoodVisualizer setpointVisualizer = new HoodVisualizer("Setpoint", Color.kGreen);

  private double setpointInches = 0.0;

  public Hood() {
    switch (Constants.currentMode) {
      case REAL:
        io = new HoodMotorTalonFX(21);

        break;
      case SIM:
        io = new HoodMotorSim(DCMotor.getKrakenX60(1), 3, 1, new PIDConstants(1, 0, 0));

        break;
      case REPLAY:
      default:
        io = new HoodMotorIO() {};
        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    Logger.processInputs("Hood", this.inputs);

    if (DriverStation.isDisabled()) {
      this.setSetpoint(0.0);
      this.io.stop();
    }

    Logger.recordOutput("Hood/SetpointInches", setpointInches);

    // Log Mechanisms
    // measuredVisualizer.update(this.inputs.masterPositionRad);
    // setpointVisualizer.update(this.setpointInches);
    // // I'm not quite sure how this works, it is semi working in sim.
    BobotState.updateHood(this.inputs.masterPositionRad);
  }

  // These needs to be reorganized

  private void setSetpoint(double setpoint) {
    setpointInches = MathUtil.clamp(setpoint, 0, 56); // not real value
    this.io.setSetpoint(this.setpointInches);
  }

  public Command setSetpointCommand(double positionInches) {
    return new InstantCommand(() -> this.setSetpoint(positionInches));
  }

  public Command setSetpointCurrentCommand() {
    return new InstantCommand(() -> this.setSetpoint(this.inputs.extentionAbsPos));
  }

  public Command pidCommand() {
    return new RunCommand(
        () -> {
          double output = this.pidController.calculate(this.inputs.extentionAbsPos);
          setVoltage(output);
        },
        this);
  }

  public Command setHoodPosition() {
    return new RunCommand(
        () ->
            this.io.setHoodPosition(
                MathUtil.clamp(
                    20, Constants.HoodConstants.MINHOOD, Constants.HoodConstants.MAXHOOD)));
  }

  public Command setHoodPosition2() {
    return new RunCommand(
        () ->
            this.io.setHoodPosition(
                MathUtil.clamp(
                    20, Constants.HoodConstants.MINHOOD, Constants.HoodConstants.MAXHOOD)));
  }

  public Command stopCommand() {
    return new InstantCommand(this.io::stop, this);
  }

  public void setVoltage(double voltage) {
    this.io.setHoodVelocity(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public Command setVolatageCommand(double voltage) {
    return new RunCommand(() -> this.io.setHoodVelocity(voltage), this);
  }

  public Command resetEncoder() {
    return new InstantCommand(this.io::resetEncoder, this);
  }

  public Command setPercentOutputCommand(double velocityRotPerSecond) {
    setpointInches = velocityRotPerSecond * 1000;
    return new RunCommand(() -> this.io.setPercentOutput(velocityRotPerSecond), this);
  }

  /* Adding new commands down here to ease readability,
   * at some point the above commands will be reorganized.
   * Triggers might also be separated at a later date, potentially added to BobotState
   */

}
