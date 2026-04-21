package org.sciborgs1155.robot.turret;

public interface TurretIO {
  public double position();

  public double velocity();

  public double acceleration();

  public void setVoltage(double voltage);
}
