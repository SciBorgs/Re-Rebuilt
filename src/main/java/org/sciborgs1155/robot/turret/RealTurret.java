package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.disabled;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.TalonUtils;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

import static org.sciborgs1155.robot.Ports.Turret.*;

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
    motor = new TalonFX(MOTOR, "shooting");

    config = new TalonFXConfiguration();

    encoderA = new CANcoder(ENCODER_A, "shooting");
    encoderB = new CANcoder(ENCODER_B, "shooting");

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

    configA.MagnetSensor.MagnetOffset = -0.464599609375;
    configB.MagnetSensor.MagnetOffset = -0.9462890625;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.Feedback.SensorToMechanismRatio = GEARING;

    motor.getConfigurator().apply(config);
    encoderA.getConfigurator().apply(configA);
    encoderB.getConfigurator().apply(configB);

    motor.setPosition(0);
    TalonUtils.addMotor(motor);
    FaultLogger.register(motor);
    FaultLogger.register(encoderA);
    FaultLogger.register(encoderB);
  }

  // @Logged
  public Optional<Angle> crtAngle(){
    return easyCRTSolver.getAngleOptional();
  }

  @Override
  @Logged
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
  
  @Logged
  public double current(){
    motor.getMotorVoltage().getValueAsDouble();
    return motor.getSupplyCurrent().getValueAsDouble();
  }

  public void periodic (){
    easyCRTSolver.getAngleOptional().
      ifPresent(
        (angle) ->
        {LoggingUtils.log("Robot/Turret/crtAngle", angle.in(Radians));
        if(disabled().getAsBoolean()) motor.setPosition(angle.in(Radians));
      });
  }
}
