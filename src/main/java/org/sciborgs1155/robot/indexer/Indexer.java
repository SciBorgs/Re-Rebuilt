package org.sciborgs1155.robot.indexer;

import static edu.wpi.first.units.Units.Amps;

import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.hopper.HopperConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Indexer extends SubsystemBase {

    private final SimpleMotor motor;
    public Trigger blocked;

    public static Indexer create() {
        
        return Robot.isReal() 
        ? new Indexer(real(), Beambreak.real(org.sciborgs1155.robot.Ports.Hopper.BEAMBREAK)) 
        : new Indexer(none(), Beambreak.none());

    }

    /**
     * creates a new none hopper.
     * @return a none simplemotor object 
     */
    public static SimpleMotor none() {
        return SimpleMotor.none();
    }

    /**
     * creates a new simplemotor hopper.
     * @return a talon simplemotor object 
     */
    private static SimpleMotor real() {
        TalonFX motor = new TalonFX(0);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = HopperConstants.CURRENT_LIMIT.in(Amps);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        return SimpleMotor.talon(motor, config);

    }

    private Indexer(SimpleMotor motor, Beambreak beambreak) {
        this.motor = motor;

        setDefaultCommand(stop());

    }

    public Command run(double v) {
        return Commands.run(() -> motor.set(v));
    }

    public Command stop() {
        return run(0);
    }


    
}
