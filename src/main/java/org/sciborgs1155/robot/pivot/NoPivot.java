package org.sciborgs1155.robot.pivot;

import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Radians;

public class NoPivot implements PivotIO {
    @Override
    public void setVoltage(double voltage) {}

    @Override
    public void setCurrentLimit(Current limit) {}

    @Override
    public double getPosition() {
        return MAX_ANGLE.in(Radians);
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public void close() throws Exception {}
}
