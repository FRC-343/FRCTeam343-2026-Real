package frc.robot.subsystems.Shooter;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
// import frc.robot.bobot_state2.BobotState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter() {
    switch (Constants.currentMode) {
      case REAL:
        io = new ShooterIOTalonFx(26, false);
        break;
      case SIM:
        io = new ShooterIOSim(DCMotor.getKrakenX60(1), 3, 1, new PIDConstants(1, 0, 0));
        break;
      case REPLAY:
      default:
        io = new ShooterIO() {};

        break;
    }
  }
  // test
  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    this.io.updateInputs(this.inputs);

    Logger.processInputs("Shooter", this.inputs);

    // Make sure the motor actually stops when the robot disabled
    if (DriverStation.isDisabled()) {
      this.io.stop();
    }
  }

  public Command ShooterWithNoStop() {
    return new RunCommand(() -> this.io.setPercentOutput(-.1), this);
  }

  public Command setVelocityCommand(double velocityRotPerSecond) {
    return new InstantCommand(() -> this.io.setVelocity(velocityRotPerSecond), this);
  }

  public Command setVelocityThenStopCommand(double velocityRotPerSecond) {
    return new RunCommand(() -> this.io.setVelocity(velocityRotPerSecond), this)
        .finallyDo(io::stop);
  }

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

  public Command setVelocityBeambreakCommand(double velocityRotPerSecond) {
    return new RunCommand(() -> this.io.setVelocity(velocityRotPerSecond), this);
  }

  public Command setPercentOutputCommand(double velocityRotPerSecond) {
    return new InstantCommand(() -> this.io.setPercentOutput(velocityRotPerSecond), this);
  }

  public Command setPercentOutputThenStopCommand(double percentDecimal) {
    // playMusic();
    return new RunCommand(() -> this.io.setPercentOutput(percentDecimal), this).finallyDo(io::stop);
  }

  public Command setPercentOutputThenStopCommandT1(double percentDecimal) {
    // playMusic();
    return new RunCommand(() -> this.io.setPercentOutputT1(percentDecimal), this)
        .finallyDo(io::stop);
  }

  public Command setPercentOutputBeambreakCommand(double percentDecimal, Trigger test) {
    return new RunCommand(() -> this.io.setPercentOutput(percentDecimal), this)
        .onlyWhile(test)
        .andThen(stopCommand());
  }

  public Command stopCommand() {
    return new InstantCommand(this.io::stop, this);
  }

  // For testing and sim

  public void playMusic() {
    this.io.playMusic();
  }

  public void pauseMusic() {
    this.io.pauseMusic();
  }
}
