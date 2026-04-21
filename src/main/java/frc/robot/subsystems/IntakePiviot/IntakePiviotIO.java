package frc.robot.subsystems.IntakePiviot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePiviotIO {

  @AutoLog
  public static class IntakePiviotMotorIOInputs {
    public boolean masterConnected;
    public double masterPositionRot;
    public double masterVelocityRadPerSec = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterCurrentAmps = 0.0;

    public Rotation2d extentionPos = new Rotation2d();

    public double encoderPos = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakePiviotMotorIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setIntakePiviotOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setIntakePiviotPosition(double rotation) {}

  public default void setPercentOutput(double percentDecimal) {}

  public default void setSetpoint(double setpoint) {}

  public default void setVoltage(double voltage) {}

  public default void resetEncoder() {}

  public default void setStatorPosition(double position) {}

  public default void stop() {
    setVelocity(0);
  }
}
