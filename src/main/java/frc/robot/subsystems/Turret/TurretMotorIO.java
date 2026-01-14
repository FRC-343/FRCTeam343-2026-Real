package frc.robot.subsystems.Turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretMotorIO {

  @AutoLog
  public static class TurretMotorIOInputs {
    public boolean masterConnected;
    public double masterPositionRad = 0.0;
    public double masterVelocityRadPerSec = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterCurrentAmps = 0.0;

    public double extentionAbsPos = 0.0;

    public Rotation2d extentionPos = new Rotation2d();
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretMotorIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setTurretOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setTurretVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurretPosition(double rotation) {}

  public default void setPercentOutput(double percentDecimal) {}

  public default void setSetpoint(double setpoint) {}

  public default void setVoltage(double voltage) {}

  public default void resetEncoder() {}

  public default void stop() {
    setTurretVelocity(0);
  }
}
