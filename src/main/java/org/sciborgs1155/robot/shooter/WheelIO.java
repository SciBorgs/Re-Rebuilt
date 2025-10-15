package org.sciborgs1155.robot.shooter;

import monologue.Logged;

// Defines the set of behaviors without implementation details of the WheelIO class (interface)
public interface WheelIO extends AutoCloseable, Logged {
    void setVoltage(double votage);
    double velocity(); // Radians per second
}
