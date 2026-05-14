package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Set;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public class Turret extends SubsystemBase implements AutoCloseable {
  private final TurretIO hardware;

  private final ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(10, 20), 0.02);
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

  public Turret(TurretIO hardware) {
    this.hardware = hardware;
    // setDefaultCommand(zero());

    pid.setTolerance(TOLERANCE);
    pid.setGoal(0);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(QUASISTATIC_VOLTAGE, DYNAMIC_VOLTAGE, Seconds.of(5)),
            new SysIdRoutine.Mechanism(
                voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
  }

  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  @Logged
  public double velocitySetpoint() {
    return pid.getSetpoint().velocity;
  }

  @Logged
  public double positionSetpoint() {
    return pid.getSetpoint().position;
  }

  @Logged
  public double position() {
    return hardware.position();
  }

  public double acceleration() {
    return hardware.acceleration();
  }

  double usedVoltage = 2;
  
  public void setVoltage(double voltage) {
    double usedVoltage = voltage;
    hardware.setVoltage(voltage);
  }

  @Logged
  public double voltage(){
    return usedVoltage;
  }

  public boolean atSetpoint() {
    return Math.abs(positionSetpoint() - hardware.position()) < TOLERANCE;
  }

  public void update(double angle) {
    hardware.setVoltage(
        pid.calculate(position(), MathUtil.clamp(angle, 0, 270))
            // + ff.calculateWithVelocities(velocity(), velocitySetpoint()));
            + ff.calculateWithVelocities(velocity(), velocitySetpoint()));
  }

  public Command goTo(double goal) {
    // return Commands.runOnce(() -> pid.setGoal(MathUtil.clamp(goal, 0, 270)))
    //     .andThen(
    //         Commands.run(
    //             () ->
    //                 hardware.setVoltage(
    //                     pid.calculate(position())
    //                         // + ff.calculateWithVelocities(velocity(), velocitySetpoint())),
    //                         + ff.calculate(velocitySetpoint())),
    //             this))
    //     .until(pid::atGoal);
    return run(() -> update(goal));
  }

  public Test goToTest(double angle) {
    Command testCommand = goTo(angle).until(pid::atGoal).withTimeout(5);
    EqualityAssertion atGoal = Assertion.eAssert("Slapdown angle", () -> angle, hardware::position);
    return new Test(testCommand, Set.of(atGoal));
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

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
