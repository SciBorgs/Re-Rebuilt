package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class TurretConstants {
  public static double kP = 1;
  public static double kI = 0;
  public static double kD = 0;
  public static double kS = 0;
  public static double kG = 0;
  public static double kA = 0.1;
  public static double kV = 1;

  public static DCMotor GEARBOX = DCMotor.getNEO(2);

  public static double TOLERANCE = 0.1;
  public static Velocity<VoltageUnit> QUASISTATIC_VOLTAGE = Volts.per(Second).of(2);
  public static Voltage DYNAMIC_VOLTAGE = Volts.of(3);

  public static double MIN_ANGLE = 0;
  public static double MAX_ANGLE = 270;

  public static Current CURRENT_LIMIT = Amps.of(12);
  public static double GEARING = 20;

  public static double MOI = 0.02;
}
