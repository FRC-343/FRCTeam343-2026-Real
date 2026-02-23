package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Turret.TurretMotorIO.TurretMotorIOInputs;

public class TurretMotorTalonFX implements TurretMotorIO {
  private final TalonFX talon;
  private final CANcoder r13;
  private final CANcoder r17;

  // private final SparkBase encoder = new SparkMax(25, null);
  // private final AbsoluteEncoder absEnc;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Current> current;

  private final StatusSignal<Angle> r13Abspos;
  private final StatusSignal<Angle> r17Abspos;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final MotionMagicVoltage Vrequest = new MotionMagicVoltage(0);

  private final Orchestra m_orchestra = new Orchestra();

  public TurretMotorTalonFX(int deviceId, int deviceId2, int deviceId3) {
    talon = new TalonFX(deviceId);
    r13 = new CANcoder(deviceId2);
    r17 = new CANcoder(deviceId3);
    voltage = talon.getMotorVoltage();
    dutyCycle = talon.getDutyCycle();
    velocity = talon.getVelocity();
    position = talon.getPosition();
    current = talon.getStatorCurrent();
    r13Abspos = r13.getAbsolutePosition();
    r17Abspos = r17.getAbsolutePosition();

    // absEnc = encoder.getAbsoluteEncoder();

    this.m_orchestra.addInstrument(talon);
    this.m_orchestra.loadMusic("output.chrp");
    talon
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(
                    new Slot0Configs().withKV(0.12).withKA(.01).withKP(10).withKI(0).withKD(.1))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(75)
                        .withMotionMagicCruiseVelocity(75)
                        .withMotionMagicJerk(200)));
    velocityVoltage.Slot = 0;

    r13.getConfigurator().apply(new CANcoderConfiguration());
    r17.getConfigurator().apply(new CANcoderConfiguration());

    StatusSignal.setUpdateFrequencyForAll(
        10, voltage, dutyCycle, velocity, position, current, r13Abspos, r17Abspos);
    talon.optimizeBusUtilization();
  }

  public void updateInputs(TurretMotorIOInputs inputs) {
    StatusSignal.refreshAll(velocity, dutyCycle, voltage, position, r13Abspos, r17Abspos);
    inputs.masterAppliedVolts = voltage.getValueAsDouble();
    inputs.masterVelocityRadPerSec = velocity.getValueAsDouble();
    inputs.masterPositionRad = position.getValueAsDouble();
    inputs.masterCurrentAmps = current.getValueAsDouble();

    inputs.r13Abspos = r13Abspos.getValueAsDouble();
    inputs.r17Abspos = r17Abspos.getValueAsDouble();
  }

  @Override
  public void setTurretVelocity(double velocityRotPerSecond) {
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
  public void setTurretPosition(double rotation) {
    talon.setControl(Vrequest.withPosition(rotation));
  }

  @Override
  public void resetEncoder() {
    talon.setPosition(0);
  }
}
