package org.sciborgs1155.robot.hood;

public class SimHood implements HoodIO {

    public SimHood() {
        return;
    }

    @Override
    public double angle() {
       return 0.0;
    }

    @Override
    public void setVoltage(double voltage) {
        return;
    }

    @Override
    public double velocity() {
        return 0.0;
    }

    @Override
    public void close() throws Exception {
        return;
    }
    
}