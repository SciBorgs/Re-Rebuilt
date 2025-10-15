package org.sciborgs1155.robot.pivot;

import edu.wpi.first.units.measure.Current;
import monologue.Logged;

public interface PivotIO extends AutoCloseable, Logged {
    public void setVoltage(double voltage);
    public void setCurrentLimit(Current limit);
    public double getPosition();
    public double getVelocity();
}
