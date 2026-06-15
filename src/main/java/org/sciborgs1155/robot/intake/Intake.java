package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.sciborgs1155.lib.SimpleMotor;
import org.sciborgs1155.robot.Robot;

public class Intake extends SubsystemBase implements AutoCloseable {
  SimpleMotor hardware;

  public Intake(SimpleMotor hardware) {
    this.hardware = hardware;
    setDefaultCommand(null);
  }

  public static Intake create() {
    return Robot.isReal() ? new Intake(realMotor()) : none();
  }

  private static SimpleMotor realMotor() {
    TalonFX motor = new TalonFX(0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
    config.Feedback.SensorToMechanismRatio = GEARING;
    return SimpleMotor.talon(motor, config);
  }

  public static Intake none() {
    return new Intake(SimpleMotor.none());
  }

  public Command spin(double power) {
    return run(() -> hardware.set(power));
  }

  public Command intake() {
    return spin(INTAKE_POWER);
  }

  public Command outtake() {
    return spin(-INTAKE_POWER);
  }

  public Command stop() {
    return spin(0);
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
