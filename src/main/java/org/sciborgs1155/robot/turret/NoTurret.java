package org.sciborgs1155.robot.turret;

public class NoTurret implements TurretIO {
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
  public void setVoltage(double voltage) {}
}
