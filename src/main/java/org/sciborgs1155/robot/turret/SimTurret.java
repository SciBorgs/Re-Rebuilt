package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimTurret implements TurretIO {
  private final DCMotorSim turret =
      new DCMotorSim(
          // LinearSystemId.createDCMotorSystem(GEARBOX, MOI, GEARING), GEARBOX, 0.01, -0.01);
          LinearSystemId.createDCMotorSystem(GEARBOX, MOI, GEARING), GEARBOX);

  public SimTurret(){
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

  private double usedVoltage = 0;

  @Override
  public void setVoltage(double voltage) {
    // usedVoltage = voltage;
    turret.setInputVoltage(voltage);
    turret.update(PERIOD.in(Seconds));
  }

  // public void update(double time) {
  //   turret.setInputVoltage(usedVoltage);
  //   turret.update(PERIOD.in(Seconds));
  // }

  @Override
  public void close() throws Exception {
    // turret.setInputVoltage(0);
  }
}
