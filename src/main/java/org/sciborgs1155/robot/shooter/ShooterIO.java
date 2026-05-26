package org.sciborgs1155.robot.shooter;

public interface ShooterIO {
    /**
     * Sets the voltage of the flywheel.
     * 
     * @param voltage desired voltage of the flywheel
     */
    void setVoltage(double voltage);

    /**
     * Gets the velocity of the flywheel in radians per second.
     * 
     * @return The velocity of the flywheel in radians per second.
     */
    double velocity();
}
