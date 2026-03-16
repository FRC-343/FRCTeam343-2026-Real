package frc.robot.subsystems.Shooter;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.bobot_state2.BobotState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter() {
    switch (Constants.currentMode) {
      case REAL:
        io = new ShooterIOTalonFx(22, 23, true);
        // 22 is inverted
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

    BobotState.updateShooterRPM(this.inputs.velocityRotPerSecond * 60);
    BobotState.updateShooterRPS(this.inputs.velocityRotPerSecond);

    Logger.processInputs("Shooter", this.inputs);

    // Make sure the motor actually stops when the robot disabled
    if (DriverStation.isDisabled()) {
      this.io.stop();
    }
  }

  public Command setVelocityThenStopCommand() {
    return new RunCommand(
            () -> this.io.setVelocity(MathUtil.clamp(BobotState.getWantedShooterRPS(), 25.0, 45.0)),
            this)
        .finallyDo(io::stop);
  }

  public Command setVelocityThenStopCommand2(double speed) {
    return new RunCommand(() -> this.io.setVelocity(speed), this).finallyDo(io::stop);
  }

  public Command setPercentOutputThenStopCommand(double percentDecimal) {
    return new RunCommand(() -> this.io.setPercentOutput(percentDecimal), this).finallyDo(io::stop);
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
