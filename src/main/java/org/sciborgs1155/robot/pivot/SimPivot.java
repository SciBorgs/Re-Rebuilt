package org.sciborgs1155.robot.pivot;

import static org.sciborgs1155.robot.pivot.PivotConstants.*;
import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.Constants.PERIOD;



import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimPivot implements PivotIO {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getNEO(4), MOI.in(KilogramSquareMeters), 1.0 / MOTOR_GEARING), // motor, why is MOI not declared
        DCMotor.getNEO(4), // velocity
        1.0 / MOTOR_GEARING,
        -LENGTH.in(Meters),
        MIN_ANGLE.in(Radians),
        MAX_ANGLE.in(Radians),
        true,
        STARTING_ANGLE.in(Radians));

    @Override
    public void setVoltage(double voltage) {
        sim.setInputVoltage(voltage);
        sim.update(PERIOD.in(Seconds));
    }

    @Override
    public void setCurrentLimit(Current limit) {}

    @Override
    public double getPosition() {
        return sim.getAngleRads();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityRadPerSec();
    }

    @Override
    public void close() throws Exception {}
}
