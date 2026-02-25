package frc.robot.subsystems.Spindexer;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.bobot_state2.BobotState;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;

  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer() {
    switch (Constants.currentMode) {
      case REAL:
        io = new SpindexerIOTalonFX(14, false);
        break;
      case SIM:
        io = new SpindexerIOSim(DCMotor.getKrakenX60(1), 3, 1, new PIDConstants(1, 0, 0));
        break;
      case REPLAY:
      default:
        io = new SpindexerIO() {};

        break;
    }
  }
  // test
  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    this.io.updateInputs(this.inputs);

    Logger.processInputs("Spindexer", this.inputs);

    // Make sure the motor actually stops when the robot disabled
    if (DriverStation.isDisabled()) {
      this.io.stop();
    }
  }

  public Command setVelocityThenStopCommand(double velocityRotPerSecond) {
    return new RunCommand(() -> this.io.setVelocity(velocityRotPerSecond), this)
        .finallyDo(io::stop);
  }

  public Command stopCommand() {
    return new InstantCommand(this.io::stop, this);
  }

  // Leaving these here for now, will see if we need them for auto later
  public Command runForTime(double speed, double time) { // -.5 for out .5 for in
    return new RunCommand(() -> this.io.setPercentOutput(speed), this)
        .withTimeout(time)
        .andThen(io::stop);
  }

  public Command runForTimeT1(double speed, double time) { // -.5 for out .5 for in
    return new RunCommand(() -> this.io.setPercentOutputT1(speed), this)
        .withTimeout(time)
        .andThen(io::stop);
  }
}
