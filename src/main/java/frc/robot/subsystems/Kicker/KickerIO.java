package frc.robot.subsystems.Kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double appliedVoltage = 0.0;
    public double appliedDutyCycle = 0.0;
    public double velocityRotPerSecond = 0.0;
    public double currentAmperage = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setVelocity(double velocityRotPerSecond) {}

  public default void setPercentOutput(double percentDecimal) {}

  public default void setVoltage(double voltage) {}

  public default void setPercentOutputT1(double percentDecimal) {}

  public default void playMusic() {}

  public default void pauseMusic() {}

  public default void stop() {
    setVoltage(0.0);
  }
}
