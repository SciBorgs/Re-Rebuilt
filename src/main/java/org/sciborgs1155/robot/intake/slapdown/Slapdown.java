package org.sciborgs1155.robot.intake.slapdown;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.*;
import static org.sciborgs1155.robot.intake.slapdown.SlapdownConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Set;
import org.sciborgs1155.lib.Assertion;
import org.sciborgs1155.lib.Assertion.EqualityAssertion;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

@Logged
public class Slapdown extends SubsystemBase implements AutoCloseable {

  private final SlapdownIO hardware;

  private final ProfiledPIDController pid = new ProfiledPIDController(P, I, D, CONSTRAINTS);

  private final ArmFeedforward ff = new ArmFeedforward(S, G, V, A);

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/slapdown/tuningP", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/slapdown/tuningI", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/slapdown/tuningD", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/slapdown/tuningS", S);
  @NotLogged private final DoubleEntry tuningG = Tuning.entry("Robot/tuning/slapdown/tuningG", G);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/slapdown/tuningV", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/slapdown/tuningA", A);

  private double pidOutput;
  private double ffOutput;
  private double appliedVoltage;

  private final SysIdRoutine sysIdRoutine;

  public Slapdown(SlapdownIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));
    pid.reset(hardware.position());
    pid.setGoal(START_ANGLE.in(Radians));

    setDefaultCommand(hold());

    sysIdRoutine =
        new SysIdRoutine(
            new Config(RAMP_RATE, STEP_VOLTAGE, TIME_OUT),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));

    SmartDashboard.putData(
        "Robot/slapdown/quasistatic forward",
        sysIdRoutine.quasistatic(Direction.kForward).withName("slapdown quasistatic forward"));

    SmartDashboard.putData(
        "Robot/slapdown/quasistatic backward",
        sysIdRoutine.quasistatic(Direction.kReverse).withName("slapdown quasistatic backward"));

    SmartDashboard.putData(
        "Robot/slapdown/dynamic forward",
        sysIdRoutine.dynamic(Direction.kForward).withName("slapdown dynamic forward"));

    SmartDashboard.putData(
        "Robot/slapdown/dynamic backward",
        sysIdRoutine.dynamic(Direction.kReverse).withName("slapdown dynamic backward"));
  }

  public static Slapdown create() {
    return new Slapdown(Robot.isReal() ? new RealSlapdown() : new SimSlapdown());
  }

  public static Slapdown none() {
    return new Slapdown(new NoSlapdown());
  }

  public Command goTo(double angle) {
    return runOnce(() -> pid.reset(hardware.position()))
        .andThen(run(() -> update(angle)))
        .withName("go to angle");
  }

  public Command extend() {
    return goTo(MIN_ANGLE.in(Radians));
  }

  public Command retract() {
    return goTo(MAX_ANGLE.in(Radians));
  }

  public Command hold() {
    return run(() -> update(pid.getGoal().position)).withName("hold");
  }

  public double position() {
    return hardware.position();
  }

  public double setpoint() {
    return pid.getSetpoint().position;
  }

  public double velocity() {
    return hardware.velocity();
  }

  public boolean atPosition(double angle) {
    return Math.abs(angle - position()) < POSITION_TOLERANCE.in(Radians);
  }

  public boolean atGoal() {
    return pid.atGoal();
  }

  public void update(double angle) {
    double rads = MathUtil.clamp(angle, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));

    pidOutput = pid.calculate(position(), rads);

    ffOutput = ff.calculate(position(), pid.getSetpoint().velocity);

    appliedVoltage = MathUtil.clamp(pidOutput + ffOutput, -MAX_VOLTAGE, MAX_VOLTAGE);

    hardware.setVoltage(appliedVoltage);
  }

  public Test goToTest(double angle) {
    EqualityAssertion atGoal =
        Assertion.eAssert(
            "Slapdown angle", () -> angle, this::position, POSITION_TOLERANCE.in(Radians));

    Command testCommand = goTo(angle).until(pid::atGoal).withTimeout(5);

    return new Test(testCommand, Set.of(atGoal));
  }

  @Override
  public void periodic() {
    hardware.update();

    if (TUNING) {
      pid.setP(tuningP.get());
      pid.setI(tuningI.get());
      pid.setD(tuningD.get());

      ff.setKs(tuningS.get());
      ff.setKg(tuningG.get());
      ff.setKv(tuningV.get());
      ff.setKa(tuningA.get());
    }
  }

  public void stop() {
    hardware.stop();
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
