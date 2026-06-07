package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.Constants.TUNING;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.ControlConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.LoggingUtils;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.Robot;

public class Shooter extends SubsystemBase {
  private final ShooterIO hardware;

  ProfiledPIDController pid =
      new ProfiledPIDController(
          P, I, D, new TrapezoidProfile.Constraints(MAX_ACCELERATION, MAX_JERK));
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(S, V, A, PERIOD.in(Seconds));
  private final SysIdRoutine characterization;

  @NotLogged private final DoubleEntry tuningP = Tuning.entry("Robot/tuning/shooter/K_P", P);
  @NotLogged private final DoubleEntry tuningI = Tuning.entry("Robot/tuning/shooter/K_I", I);
  @NotLogged private final DoubleEntry tuningD = Tuning.entry("Robot/tuning/shooter/K_D", D);
  @NotLogged private final DoubleEntry tuningS = Tuning.entry("Robot/tuning/shooter/S", S);
  @NotLogged private final DoubleEntry tuningV = Tuning.entry("Robot/tuning/shooter/V", V);
  @NotLogged private final DoubleEntry tuningA = Tuning.entry("Robot/tuning/shooter/A", A);

  /**
   * Returns the shooter subsystem
   *
   * @return Creates real or simulated shooter based on {@link Robot#isReal()}.
   */
  public static Shooter create() {
    return Robot.isReal() ? new Shooter(new RealShooter()) : new Shooter(new SimShooter());
  }

  public static Shooter none() {
    return new Shooter(new NoShooter());
  }

  public Shooter(ShooterIO hardware) {
    this.hardware = hardware;
    pid.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    characterization =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(10), 
                Seconds.of(10),
                (state) -> SignalLogger.writeString("shooter state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                v -> hardware.setVoltage(v.in(Volts)), null, this, "shooter"
            )
        );

    SmartDashboard.putData(
        "Robot/shooter/shooter quasistatic backward",
        characterization.quasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "Robot/shooter/shooter quasistatic forward",
        characterization.quasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Robot/shooter/shooter dynamic backward", characterization.dynamic(Direction.kReverse));
    SmartDashboard.putData(
        "Robot/shooter/shooter dynamic forward", characterization.dynamic(Direction.kForward));

    setDefaultCommand(idleShooter());
  }

  public void update(double velocitySetpoint, boolean noDeceleration) {
    double velocity =
        MathUtil.clamp(
            velocitySetpoint,
            -MAX_VELOCITY.in(RadiansPerSecond),
            MAX_VELOCITY.in(RadiansPerSecond));
    double lastVelocity = pid.getSetpoint().position;
    double pidVolts = pid.calculate(velocity(), velocity);
    double ffVolts = feedforward.calculateWithVelocities(lastVelocity, pid.getSetpoint().position);
    double volts = MathUtil.clamp(pidVolts + ffVolts, -MAX_VOLTAGE, MAX_VOLTAGE);
    hardware.setVoltage(noDeceleration ? Math.max(0, volts) : volts);
  }

  public void update(double velocitySetpoint) {
    update(velocitySetpoint, false);
  }

  public Double velocity() {
    return hardware.velocity();
  }

  public boolean atVelocity(double Velocity) {
    return Math.abs(Velocity - velocity()) <= VELOCITY_TOLERANCE.in(RadiansPerSecond);
  }

  public double setpoint() {
    return pid.getSetpoint().position;
  }

  public Boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble())).withName("running shooter");
  }

  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

  public Command idleShooter() {
    return run(() -> update(IDLE_VELOCITY.in(RadiansPerSecond))).withName("idle shooter");
  }

  public Command manualShooter(InputStream input) {
    return runShooter(
            input
                .deadband(.15, 1)
                .scale(MAX_VELOCITY.in(RadiansPerSecond))
                .scale(PERIOD.in(Seconds))
                .add(() -> pid.getSetpoint().position))
        .withName("manual shooter");
  }

  public void close() throws Exception {
    hardware.close();
  }

  @Override
  public void periodic() {
    var command = getCurrentCommand();
    LoggingUtils.log("Robot/shooter/current command", command != null ? command.getName() : "None");
    LoggingUtils.log("Robot/shooter/velocity", velocity());
    if (TUNING) {
      pid.setP(tuningP.get());
      pid.setI(tuningI.get());
      pid.setD(tuningD.get());
      feedforward.setKs(tuningS.get());
      feedforward.setKv(tuningV.get());
      feedforward.setKa(tuningA.get());
    }
  }
}
