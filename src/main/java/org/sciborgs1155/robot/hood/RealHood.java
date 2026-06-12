package org.sciborgs1155.robot.hood; 

import static org.sciborgs1155.robot.Ports.Hood.MOTOR_PORT;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import org.sciborgs1155.lib.FaultLogger;
import org.sciborgs1155.lib.TalonUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class RealHood implements HoodIO {
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    public RealHood() {
        motor = new TalonFX(MOTOR_PORT);
        config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = GEARING;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motor.getConfigurator().apply(config);
        TalonUtils.addMotor(motor);
        FaultLogger.register(motor);
    }
    
    @Override
    public double angle() {
        return motor.getPosition().getValue().in(Radians);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public double velocity() {
        return motor.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        motor.close();
    }
}