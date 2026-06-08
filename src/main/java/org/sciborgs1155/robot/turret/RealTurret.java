package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class RealTurret implements TurretIO {
  private final TalonFX motor;

  private final TalonFXConfiguration config;

  private final CANcoder encoderA;
  private final CANcoder encoderB;

  private final CANcoderConfiguration configA;
  private final CANcoderConfiguration configB;

  private final EasyCRTConfig easyCRTConfig;

  private final EasyCRT easyCRTSolver;

  // TODO: Config

  public RealTurret() {
    motor = new TalonFX(2);

    config = new TalonFXConfiguration();

    encoderA = new CANcoder(5); // get real number
    encoderB = new CANcoder(8); // get real number

    configA = new CANcoderConfiguration();
    configB = new CANcoderConfiguration();

    // encoderAAngle = () -> encoderA.getAbsolutePosition().getValue()

    easyCRTConfig =
        new EasyCRTConfig(
                (() -> encoderA.getAbsolutePosition().getValue()),
                (() -> encoderB.getAbsolutePosition().getValue()))
            .withMatchTolerance(CRT_MATCH_TOLERANCE);
    easyCRTConfig.withEncoderRatios(
        TURRET_GEARING / ENCODER_A_GEARING, TURRET_GEARING / ENCODER_B_GEARING);

    easyCRTSolver = new EasyCRT(easyCRTConfig);

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

  @Override
  public void close() throws Exception {
    motor.setVoltage(0);
  }
}
