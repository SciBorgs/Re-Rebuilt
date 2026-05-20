package org.sciborgs1155.robot.hopper;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;

import org.sciborgs1155.lib.Beambreak;
import org.sciborgs1155.lib.SimpleMotor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.hopper.HopperConstants;


public class Hopper extends SubsystemBase {
    private final SimpleMotor motor;
    private final Beambreak beambreak;
    public Trigger blocked;

    /**
     * creates a real/none hoppper depending on if connected to robot.
     * @return real/none hopper object
     */
    private Hopper create() {
        
        return Robot.isReal() 
        ? new Hopper(real(), Beambreak.real(org.sciborgs1155.robot.Ports.Hopper.BEAMBREAK)) 
        : new Hopper(none(), Beambreak.none());

    }

    /**
     * creates a new none hopper.
     * @return a none simplemotor object 
     */
    private static SimpleMotor none() {
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


    private Hopper(SimpleMotor motor, Beambreak beambreak) {
        this.motor = motor;
        this.beambreak = beambreak;

        this.blocked = new Trigger(() -> beambreak.get());

        setDefaultCommand(stop());
    }

    /**
     * runs motor at a set voltage.
     * @return a command setting power of motor to voltage
     */
    public Command run(double voltage) {
        return Commands.run(() -> motor.set(voltage));
    }

    /**
     * stops motor.
     */
    public Command stop() {
        return run(0);
    }

}
