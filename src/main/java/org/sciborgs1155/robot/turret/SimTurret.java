package org.sciborgs1155.robot.turret;

import edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants.*;

public class SimTurret implements TurretIO {
  private final DCMotorSim turret = new DCMotorSim(null, GEARBOX, 0.01);

  public SimTurret() {
    turret.update(0);
  }

  @Override
  public double position() {
    return turret.getAngularPositionRad();
  }

  @Override
  public double velocity() {
    return turret.getAngularVelocityRadPerSec();
  }

  @Override
  public double acceleration() {
    return turret.getAngularAccelerationRadPerSecSq();
  }

  @Override
  public void setVoltage(double voltage) {
    turret.setInputVoltage(voltage);
    turret.update(org.sciborgs1155.robot.Constants.PERIOD.in(Seconds));
  }
}
