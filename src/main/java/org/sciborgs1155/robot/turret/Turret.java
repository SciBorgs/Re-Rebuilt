package org.sciborgs1155.robot.turret;

import org.sciborgs1155.lib.CommandRobot;
import org.sciborgs1155.robot.Robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static org.sciborgs1155.robot.turret.TurretConstants.*;

public class Turret extends CommandRobot {
  private final TurretIO hardware;

  private final ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, null);
  private final TurretFeedForward ff = new TurretFeedForward(kS, kG, kV, kA);

  public static Turret create() {
    return new Turret(Robot.isReal() ? new RealTurret() : new SimTurret());
  }

  public static Turret none() {
    return new Turret(new NoTurret());
  }

  private Turret(TurretIO hardware) {
    this.hardware = hardware;
    setDefaultCommand(zero());
  }

  private final double velocity(){
    return hardware.velocity();
  }

  private final double velocitySetpoint(){
    return pid.getSetpoint().velocity;
  }

  private final double positionSetpoint(){
    return pid.getSetpoint().position;
  }
  
  private final double position(){
    return hardware.position();
  }

  private final double acceleration(){
    return hardware.acceleration();
  }

  private void setVoltage(double voltage){
    hardware.setVoltage(voltage);
  }

  public boolean atSetpoint(){
    return Math.abs(positionSetpoint() - hardware.position()) < TOLERANCE;
  }

  public Command goTo(double goal){
    return run(() -> ff.calculate(pid.getSetpoint()));
  }

  public Command zero(){
    return run(() -> goTo(0));
  }

  private void place(double position){
    double goal = double.isNaN(position) ? 0.0 : MathUtil.clamp(position, 0.0, 270.0);
  }
}
