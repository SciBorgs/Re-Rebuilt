package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public class Turret extends SubsystemBase {
  private final TurretIO hardware;

  private final ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(0.5, 5));
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  private final SysIdRoutine sysIdRoutine;

  // TODO: tune, sysid, test

  public static Turret create() {
    return new Turret(Robot.isReal() ? new RealTurret() : new SimTurret());
  }

  public static Turret none() {
    return new Turret(new NoTurret());
  }

  private final DoubleEntry S = Tuning.entry("/Robot/tuning/turret/kS", kS);
  private final DoubleEntry V = Tuning.entry("/Robot/tuning/turret/kV", kV);
  private final DoubleEntry A = Tuning.entry("/Robot/tuning/turret/kA", kA);
  private final DoubleEntry P = Tuning.entry("/Robot/tuning/turret/kP", kP);
  private final DoubleEntry I = Tuning.entry("/Robot/tuning/turret/kI", kI);
  private final DoubleEntry D = Tuning.entry("/Robot/tuning/turret/kD", kD);

  private Turret(TurretIO hardware) {
    this.hardware = hardware;
    setDefaultCommand(zero());

    pid.setTolerance(TOLERANCE);
    pid.setGoal(0);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(QUASISTATIC_VOLTAGE, DYNAMIC_VOLTAGE, Seconds.of(5)),
            new SysIdRoutine.Mechanism(
                voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
  }

  private final double velocity() {
    return hardware.velocity();
  }

  private final double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  private final double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  private final double position() {
    return hardware.position();
  }

  private final double acceleration() {
    return hardware.acceleration();
  }

  private void setVoltage(double voltage) {
    hardware.setVoltage(voltage);
  }

  public boolean atSetpoint() {
    return Math.abs(positionSetpoint() - hardware.position()) < TOLERANCE;
  }

  public void update(double angle) {
    hardware.setVoltage(
        pid.calculate(position(), MathUtil.clamp(angle, 0, 270))
            + ff.calculateWithVelocities(velocity(), velocitySetpoint()));
  }

  public Command goTo(double goal) {
    return Commands.runOnce(() -> pid.setGoal(MathUtil.clamp(goal, 0, 270)))
        .andThen(
            Commands.run(
                () ->
                    hardware.setVoltage(
                        pid.calculate(position())
                            + ff.calculateWithVelocities(velocity(), velocitySetpoint())),
                this))
        .until(this::atSetpoint);
  }

  public Command zero() {
    return goTo(0);
  }

  public void periodic() {
    if (TUNING) {
      pid.setP(P.get());
      pid.setI(I.get());
      pid.setD(D.get());
      ff.setKs(S.get());
      ff.setKa(A.get());
      ff.setKv(V.get());
    }
  }
}
