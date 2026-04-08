package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
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

public class ShooterIOTalonFx implements ShooterIO {
  private final TalonFX talon;
  private final TalonFX follower;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<AngularVelocity> velocity;

  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Double> followerDutyCycle;
  private final StatusSignal<AngularVelocity> followerVelocity;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final Orchestra m_orchestra = new Orchestra();

  public ShooterIOTalonFx(int deviceId, int deviceId2, boolean isInverted) {
    talon = new TalonFX(deviceId);
    follower = new TalonFX(deviceId2);
    voltage = talon.getMotorVoltage();
    dutyCycle = talon.getDutyCycle();
    velocity = talon.getVelocity();
    followerVoltage = follower.getMotorVoltage();
    followerDutyCycle = follower.getDutyCycle();
    followerVelocity = follower.getVelocity();

    this.m_orchestra.addInstrument(talon);
    this.m_orchestra.loadMusic("output2.chrp");

    talon
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(
                            isInverted
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive))
                .withSlot0(
                    new Slot0Configs().withKV(.11).withKP(.1).withKI(0).withKD(0.0).withKS(1.2)));
    follower
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                .withSlot0(
                    new Slot0Configs().withKV(.11).withKP(.1).withKI(0).withKD(0.0).withKS(1.2)));
    velocityVoltage.Slot = 0;

    StatusSignal.setUpdateFrequencyForAll(
        10, voltage, dutyCycle, velocity, followerVoltage, followerDutyCycle, followerVelocity);
    talon.optimizeBusUtilization();
    follower.optimizeBusUtilization();

    follower.setControl(new Follower(talon.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void updateInputs(ShooterIOInputs inputs) {
    StatusSignal.refreshAll(
        velocity, dutyCycle, voltage, followerVoltage, followerDutyCycle, followerVelocity);
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.appliedDutyCycle = dutyCycle.getValueAsDouble();
    inputs.velocityRotPerSecond = velocity.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRotPerSecond) {
    talon.setControl(velocityVoltage.withVelocity(velocityRotPerSecond));
    // follower.setControl(velocityVoltage.withVelocity(velocityRotPerSecond * 3.0));
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
    // follower.setVoltage(voltage);
  }
}
