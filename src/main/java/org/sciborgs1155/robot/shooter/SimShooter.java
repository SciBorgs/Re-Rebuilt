package org.sciborgs1155.robot.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.shooter.ShooterConstants.ControlConstants.*;

public class SimShooter implements ShooterIO {
  private final FlywheelSim shooter;

  /** Creates an instance of the flywheel motor. */
  public SimShooter() {
    shooter =
        new FlywheelSim(LinearSystemId.identifyVelocitySystem(V, A), DCMotor.getKrakenX60(2));
  }

  @Override
  public void setVoltage(double voltage) {
    shooter.setInputVoltage(voltage);
    shooter.update(PERIOD.in(Seconds));
  }

  @Override
  public double velocity() {
    return shooter.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() throws Exception {}
}
