package org.sciborgs1155.robot.intake.slapdown;

public class NoSlapdown implements SlapdownIO {

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double position() {
    return 0.0;
  }

  @Override
  public double velocity() {
    return 0.0;
  }
}
