package org.sciborgs1155.robot.hood;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.hood.HoodConstants.GEARING;
import static org.sciborgs1155.robot.hood.HoodConstants.HOOD_RADIUS;
import static org.sciborgs1155.robot.hood.HoodConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.hood.HoodConstants.MOI;
import static org.sciborgs1155.robot.hood.HoodConstants.STARTING_ANGLE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimHood implements HoodIO {

  private final SingleJointedArmSim simulator;

  public SimHood() {
    simulator =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), // Needs to be X44
            GEARING,
            MOI,
            HOOD_RADIUS.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            STARTING_ANGLE.in(Radians));
  }

  @Override
  public double angle() {
    return simulator.getAngleRads();
  }

  @Override
  public void setVoltage(double voltage) {
    simulator.setInputVoltage(voltage);
    simulator.update(PERIOD.in(Seconds));
  }

  @Override
  public double velocity() {
    return simulator.getVelocityRadPerSec();
  }

  @Override
  public double getVoltage() {
    return simulator.getInput(0);
  }

  @Override
  public void close() throws Exception {}
}
