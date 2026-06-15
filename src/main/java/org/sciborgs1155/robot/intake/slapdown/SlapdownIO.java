package org.sciborgs1155.robot.intake.slapdown;

public interface SlapdownIO extends AutoCloseable {

  void setVoltage(double voltage);

  double position();

  // defaults to
  default double velocity() {
    return 0.0;
  }

  default void stop() {
    setVoltage(0.0);
  }

  default void update() {}

  @Override
  default void close() throws Exception {}
}
