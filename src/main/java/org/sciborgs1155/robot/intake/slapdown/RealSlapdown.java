package org.sciborgs1155.robot.intake.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.robot.Ports.Slapdown.EXTENSION;
import static org.sciborgs1155.robot.intake.IntakeConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.intake.IntakeConstants.GEARING;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

public class RealSlapdown implements SlapdownIO {

  private final TalonFX motor;

  public RealSlapdown() {
    motor = new TalonFX(EXTENSION);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = GEARING;

    motor.getConfigurator().apply(config);

    motor.setPosition(SlapdownConstants.START_ANGLE);

    TalonUtils.addMotor(motor);
    FaultLogger.register(motor);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double position() {
    return motor.getPosition().getValue().in(Radians);
  }

  @Override
  public double velocity() {
    return motor.getVelocity().getValue().in(RadiansPerSecond);
  }

  @Override
  public void stop() {
    motor.setVoltage(0);
  }

  @Override
  public void close() {
    motor.close();
  }
}
