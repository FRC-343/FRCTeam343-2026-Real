package frc.robot.subsystems.IntakePiviot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakePiviotMotorTalonFX implements IntakePiviotIO {
  private final TalonFX talon;

  private final CANcoder absEnc;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Current> current;

  private final StatusSignal<Angle> encPos;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final MotionMagicVoltage Vrequest = new MotionMagicVoltage(0);

  private final Orchestra m_orchestra = new Orchestra();

  public IntakePiviotMotorTalonFX(int deviceId, int encoderID) {
    absEnc = new CANcoder(encoderID);
    talon = new TalonFX(deviceId);
    voltage = talon.getMotorVoltage();
    dutyCycle = talon.getDutyCycle();
    velocity = talon.getVelocity();
    position = talon.getPosition();
    current = talon.getStatorCurrent();
    encPos = absEnc.getAbsolutePosition();

    this.m_orchestra.addInstrument(talon);
    this.m_orchestra.loadMusic("output.chrp");
    talon
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withSlot0(
                    new Slot0Configs()
                        .withKV(0.1)
                        .withKA(.2)
                        .withKP(16.0)
                        .withKI(0)
                        .withKD(0.02)
                        .withKS(.0))
                .withFeedback(
                    new FeedbackConfigs()
                        .withFusedCANcoder(absEnc)
                        .withRotorToSensorRatio(45)
                        .withSensorToMechanismRatio(1.33))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(5)
                        .withMotionMagicCruiseVelocity(5)
                        .withMotionMagicJerk(300)));

    velocityVoltage.Slot = 0;

    StatusSignal.setUpdateFrequencyForAll(
        10, voltage, dutyCycle, velocity, position, current, encPos);
    talon.optimizeBusUtilization();
  }

  public void updateInputs(IntakePiviotMotorIOInputs inputs) {
    StatusSignal.refreshAll(velocity, dutyCycle, voltage, position);
    inputs.masterAppliedVolts = voltage.getValueAsDouble();
    inputs.masterVelocityRadPerSec = velocity.getValueAsDouble();
    inputs.masterPositionRot = position.getValueAsDouble();
    inputs.masterCurrentAmps = current.getValueAsDouble();
    inputs.encoderPos = encPos.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRotPerSecond) {
    talon.setControl(dutyCycleOut.withOutput(velocityRotPerSecond));
    // this.follower.setControl(dutyCycleOut.withOutput(velocityRotPerSecond));
  }

  @Override
  public void setPercentOutput(double percentDecimal) {
    talon.setControl(dutyCycleOut.withOutput(percentDecimal));
    // this.follower.setControl(dutyCycleOut.withOutput(percentDecimal));
  }

  @Override
  public void setSetpoint(double setpoint) {
    talon.setControl(dutyCycleOut.withOutput(setpoint));
    // this.follower.setControl(dutyCycleOut.withOutput(setpoint));
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setControl(Vrequest.withPosition(voltage));
  }

  @Override
  public void setIntakePiviotPosition(double rotation) {
    talon.setControl(Vrequest.withPosition(rotation));
  }

  @Override
  public void resetEncoder() {
    talon.setPosition(0);
  }

  @Override
  public void setStatorPosition(double position) {
    talon.setPosition(position, 0.010);
  }
}
