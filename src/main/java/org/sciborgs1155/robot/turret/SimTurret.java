package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.turret.TurretConstants.CRT_MATCH_TOLERANCE;
import static org.sciborgs1155.robot.turret.TurretConstants.GEARBOX;
import static org.sciborgs1155.robot.turret.TurretConstants.GEARING;
import static org.sciborgs1155.robot.turret.TurretConstants.MOI;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class SimTurret implements TurretIO {
  private final DCMotorSim turret =
      new DCMotorSim(
          // LinearSystemId.createDCMotorSystem(GEARBOX, MOI, GEARING), GEARBOX, 0.01, -0.01);
          LinearSystemId.createDCMotorSystem(GEARBOX, MOI, GEARING), GEARBOX);
  private final CANcoder encoderA = new CANcoder(5);
  private final CANcoder encoderB = new CANcoder(8);

  public EasyCRT easyCRT() {
    EasyCRTConfig easyCRTConfig =
        new EasyCRTConfig(
                () -> encoderA.getAbsolutePosition().getValue(),
                () -> encoderB.getAbsolutePosition().getValue())
            .withMatchTolerance(CRT_MATCH_TOLERANCE);
    return new EasyCRT(easyCRTConfig);
  }

  public SimTurret() {
    turret.update(0);
  }

  @Override
  public double position() {
    return turret.getAngularPositionRad();
  }

  @Override
  public double velocity() {
    return turret.getAngularVelocityRadPerSec();
  }

  @Override
  public double acceleration() {
    return turret.getAngularAccelerationRadPerSecSq();
  }

  private double usedVoltage = 0;

  @Override
  public void setVoltage(double voltage) {
    encoderA.getSimState().setSupplyVoltage(voltage);
    encoderB.getSimState().setSupplyVoltage(voltage);
    turret.setInputVoltage(voltage);
    turret.update(PERIOD.in(Seconds));
  }

  @Override
  public void close() throws Exception {
    // turret.setInputVoltage(0);
  }
}
