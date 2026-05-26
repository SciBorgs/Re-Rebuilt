package org.sciborgs1155.robot.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.Ports.Shooter.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

public class RealShooter implements ShooterIO {
    private final TalonFX leader;
    private final TalonFX follower;

    public RealShooter(){
        leader = new TalonFX(LEADER);
        follower = new TalonFX(FOLLOWER);
        
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT.in(Amps);
        talonConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);

        talonConfig.Feedback.SensorToMechanismRatio = SENSOR_MECHANISM_RATIO;
        


        leader.getConfigurator().apply(talonConfig);
        follower.getConfigurator().apply(talonConfig);
    }

    @Override
    public void setVoltage(double voltage) {
        leader.setVoltage(voltage);
    }

    @Override
    public double velocity() {
        return leader.getVelocity().getValueAsDouble();
    }
    
}
