package frc.robot.subsystems.Turret;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
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

public class Turret extends SubsystemBase {
  private final TurretMotorIO io;

  boolean test = false;

  private final TurretMotorIOInputsAutoLogged inputs = new TurretMotorIOInputsAutoLogged();

  public Turret() {
    switch (Constants.currentMode) {
      case REAL:
        io = new TurretMotorTalonFX(13);

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

    // double r17 = BobotState.getR13AbsPos() * 17.0; // 13T gear → mod 17
    // double r13 = BobotState.getR17AbsPos() * 13.0; // 17T gear → mod 13

    // // Reconstruct X in [0,221)
    // double X = TurretCRT1.reconstruct(r17, r13);

    // // Convert to turret rotations
    // double turretRot = X / 221.0;

    // // Convert to radians
    // double turretRad = TurretCRT1.turretRotToRadians(turretRot);

    // // Apply zero offset (store once at calibration)
    // // turretRad -= TURRET_ZERO_OFFSET_RAD;

    // BobotState.updateTurretPos1(turretRad);
    BobotState.updateTurretPos2(this.inputs.masterPositionRot);
  }

  // Command to stop the motor
  public Command stopCommand() {
    return new InstantCommand(this.io::stop, this);
  }

  public Command resetEncoder() {
    return new InstantCommand(this.io::resetEncoder, this);
  }

  public Command setTurretPosition1() {
    return new RunCommand(() -> this.io.setTurretPosition(-BobotState.getMotorTarget1()));
  }

  // public Command setTurretPosition2() {

  //   return new RunCommand(() -> this.io.setTurretPosition(BobotState.getOptiTurretYaw()));
  // }
}
