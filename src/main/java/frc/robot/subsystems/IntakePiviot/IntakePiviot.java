package frc.robot.subsystems.IntakePiviot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/*
 * self explanatory
 * Do note that out Visulizer is not currently working as of 2/6/2025
 */

public class IntakePiviot extends SubsystemBase {
  private final IntakePiviotIO io;

  boolean test = false;

  private final IntakePiviotMotorIOInputsAutoLogged inputs =
      new IntakePiviotMotorIOInputsAutoLogged();

  public IntakePiviot() {
    switch (Constants.currentMode) {
      case REAL:
        io = new IntakePiviotMotorTalonFX(21, 14);

        break;
      case SIM:
        io = new IntakePiviotMotorSim(DCMotor.getKrakenX60(1), 3, 1, new PIDConstants(1, 0, 0));

        break;
      case REPLAY:
      default:
        io = new IntakePiviotIO() {};
        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);

    Logger.processInputs("IntakePiviot", this.inputs);

    // double r17 = BobotState.getR13AbsPos() * 17.0; // 13T gear → mod 17
    // double r13 = BobotState.getR17AbsPos() * 13.0; // 17T gear → mod 13

    // // Reconstruct X in [0,221)
    // double X = IntakePiviotCRT1.reconstruct(r17, r13);

    // // Convert to IntakePiviot rotations
    // double IntakePiviotRot = X / 221.0;

    // // Convert to radians
    // double IntakePiviotRad = IntakePiviotCRT1.IntakePiviotRotToRadians(IntakePiviotRot);

    // // Apply zero offset (store once at calibration)
    // // IntakePiviotRad -= IntakePiviot_ZERO_OFFSET_RAD;

    // BobotState.updateIntakePiviotPos1(IntakePiviotRad);
  }

  // Command to stop the motor

  public Command setVelocityThenStopCommand2(double speed) {
    return new RunCommand(() -> this.io.setVelocity(speed), this).finallyDo(io::stop);
  }

  public Command stopCommand() {
    return new InstantCommand(this.io::stop, this);
  }

  public Command resetEncoder() {
    return new InstantCommand(this.io::resetEncoder, this);
  }

  public Command setAngle(double angle) {
    return new RunCommand(() -> this.io.setIntakePiviotPosition(angle));
  }

  public Command holdAngle() {
    return new RunCommand(() -> this.io.setIntakePiviotPosition(inputs.masterPositionRot));
  }
}
