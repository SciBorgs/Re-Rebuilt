package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.hood.HoodConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.Tuning;

public class Hood extends SubsystemBase implements AutoCloseable {
  private final HoodIO hardware;

  @Logged
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          P,
          I,
          D,
          new Constraints(
              MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));

  private final ArmFeedforward ff = new ArmFeedforward(S, G, V, A);
  private final SysIdRoutine sysIdRoutine;

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/hood/tuningP", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/hood/tuningI", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/hood/tuningD", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/hood/tuningS", S);
  @NotLogged private final DoubleEntry tuningG = Tuning.entry("Robot/tuning/hood/tuningG", G);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/hood/tuningV", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/hood/tuningA", A);

  public Hood(HoodIO hardware) {
    this.hardware = hardware;

    pid.setTolerance(POSITION_TOLERANCE.in(Radians));

    sysIdRoutine =
        new SysIdRoutine(
            new Config(
                RAMP_RATE,
                STEP_VOLTAGE,
                TIME_OUT,
                (state) -> SignalLogger.writeString("slapdown state", state.toString())),
            new Mechanism(voltage -> hardware.setVoltage(voltage.in(Volts)), null, this));
    SmartDashboard.putData(
        "Robot/hood/quasistatic forward",
        sysIdRoutine
            .quasistatic(Direction.kForward)
            // .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("hood quasistatic forward"));
    SmartDashboard.putData(
        "Robot/hood/quasistatic backward",
        sysIdRoutine
            .quasistatic(Direction.kReverse)
            // .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("hood quasistatic backward"));
    SmartDashboard.putData(
        "Robot/hood/dynamic forward",
        sysIdRoutine
            .dynamic(Direction.kForward)
            // .until(() -> atPosition(MAX_ANGLE.in(Radians)))
            .withName("hood dynamic forward"));
    SmartDashboard.putData(
        "Robot/hood/dynamic backward",
        sysIdRoutine
            .dynamic(Direction.kReverse)
            // .until(() -> atPosition(MIN_ANGLE.in(Radians)))
            .withName("hood dynamic backward"));
  }

  /**
   * gets the current angle of the hood
   *
   * @return the angle in radians
   */
  @Logged
  public double getAngle() {
    return hardware.angle();
  }

  /**
   * gets the current voltage of the hood motor
   *
   * @return the voltage in volts
   */
  @Logged
  public double voltage() {
    return hardware.getVoltage();
  }

  /**
   * returns the angle setpoint of the hood
   *
   * @return the position of the setpoint
   */
  @Logged
  public double angleSetpoint() {
    return pid.getSetpoint().position;
  }

  /**
   * returns the angle goal of the hood trapezoid profile
   *
   * @return the position of the goal
   */
  @Logged
  public double angleGoal() {
    return pid.getGoal().position;
  }

  /**
   * Gets the current velocity of the hood
   *
   * @return Current velocity of the hood
   */
  @Logged
  public double velocity() {
    return hardware.velocity();
  }

  /**
   * Checks whether the hood is at a set desired state
   *
   * @return Whether or not the hood is at its desired state.
   */
  @Logged
  public boolean atGoal() {
    return Math.abs(angleGoal() - getAngle()) < POSITION_TOLERANCE.in(Radians);
  }

  private void update(double position) {
    double goal = MathUtil.clamp(position, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
    double feedback = pid.calculate(getAngle(), goal);
    double feedForward = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity);
    hardware.setVoltage(feedback + feedForward);
  }

  public Command goTo(DoubleSupplier goal) {
    return run(() -> update(goal.getAsDouble())).withName("Hood goTo");
  }

  public Command goTo(Angle goal) {
    return goTo(() -> goal.in(Radians));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
