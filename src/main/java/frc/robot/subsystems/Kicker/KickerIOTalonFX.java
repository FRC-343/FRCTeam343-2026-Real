package frc.robot.subsystems.Kicker;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class KickerIOTalonFX implements KickerIO {
  private final TalonFX talon;
  private final TalonFX talon2;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<AngularVelocity> velocity;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final Orchestra m_orchestra = new Orchestra();

  public KickerIOTalonFX(int deviceId, int id2, boolean isInverted) {
    talon = new TalonFX(deviceId);
    talon2 = new TalonFX(id2);
    voltage = talon.getMotorVoltage();
    dutyCycle = talon.getDutyCycle();
    velocity = talon.getVelocity();

    talon
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(
                            isInverted
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs().withKV(0.12).withKP(.2).withKI(0).withKD(0))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(50)
                        .withStatorCurrentLimitEnable(true)));
    velocityVoltage.Slot = 0;

    StatusSignal.setUpdateFrequencyForAll(10, voltage, dutyCycle, velocity);
    talon.optimizeBusUtilization();

    talon2.setControl(new Follower(talon.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void updateInputs(KickerIOInputs inputs) {
    StatusSignal.refreshAll(velocity, dutyCycle, voltage);
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.appliedDutyCycle = dutyCycle.getValueAsDouble();
    inputs.velocityRotPerSecond = velocity.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRotPerSecond) {
    talon.setControl(velocityVoltage.withVelocity(velocityRotPerSecond));
  }

  @Override
  public void setPercentOutput(double percentDecimal) {
    talon.setControl(dutyCycleOut.withOutput(percentDecimal));
  }

  @Override
  public void setPercentOutputT1(double percentDecimal) {
    talon.setControl(dutyCycleOut.withOutput(percentDecimal));
  }

  @Override
  public void playMusic() {
    m_orchestra.play();
  }

  @Override
  public void pauseMusic() {
    m_orchestra.pause();
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setVoltage(voltage);
  }
}
