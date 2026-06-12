package org.sciborgs1155.robot.hood;

public class NoHood implements HoodIO {

  @Override
  public double angle() {
    return 0.0;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double velocity() {
    return 0.0;
  }

  @Override
  public double getVoltage() {
    return 0.0;
  }

  @Override
  public void close() throws Exception {}
}
