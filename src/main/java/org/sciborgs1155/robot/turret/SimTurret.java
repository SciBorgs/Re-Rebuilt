package org.sciborgs1155.robot.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import org.sciborgs1155.robot.Constants.*;

public class SimTurret implements TurretIO {
  private final DCMotorSim turret = new DCMotorSim(null, gearbox, 0.01);

  public SimTurret(){
    turret.update(0);
  }

  @Override
  public double position() {
    return 0;
  }

  @Override
  public double velocity() {
    return 0;
  }

  @Override
  public double acceleration() {
    return 0;
  }

  @Override
  public void setVoltage(double voltage) {
    turret.setInputVoltage(voltage);
    turret.update(Constants.PERIOD);
  }
}
