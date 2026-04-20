package org.sciborgs1155.robot.turret;

import org.sciborgs1155.lib.FaultLogger;

import edu.wpi.first.units.Units.Radians;

import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class RealTurret implements TurretIO {
  private final TalonFX motor;

  private final TalonFXConfiguration config;

  //TODO: Config

  public RealTurret(){
    motor = new TalonFX(2);

    config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.Feedback.SensorToMechanismRatio = GEARING;

    motor.getConfigurator().apply(config);

    motor.setPosition(0);
    TalonUtils.addMotor(motor);
    FaultLogger.register(motor);
  }
  @Override
  public double position() {
    return motor.getPosition().getValueAsDouble();
  }

  @Override
  public double velocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
  public double acceleration() {
    return motor.getAcceleration().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
