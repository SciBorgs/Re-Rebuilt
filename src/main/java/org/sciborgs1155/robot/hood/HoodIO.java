package org.sciborgs1155.robot.hood;

public interface HoodIO extends AutoCloseable{
    /**
     * Gets angle of the hood
     * 
     * @return angle in Radians
     */
    double angle();

    /**
     * Sets the voltage of the motor
     * @param voltage
     */
    void setVoltage(double voltage);

    /**
     * Gets the current velocity of the hood
     * @return veolcity in rads/sec
     */
    double velocity();

    @Override
    void close() throws Exception;
}