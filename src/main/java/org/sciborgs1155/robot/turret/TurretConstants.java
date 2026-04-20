package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class TurretConstants {
  public static double kP = 0;
  public static double kI = 0;
  public static double kD = 0;
  public static double kS = 0;
  public static double kG = 0;
  public static double kA = 0;
  public static double kV = 0;

  public static DCMotor GEARBOX = DCMotor.getNEO(1);

  public static double TOLERANCE = 0.1;
  public static Velocity<VoltageUnit> QUASISTATIC_VOLTAGE = Volts.of(2);
  public static Voltage DYNAMIC_VOLTAGE = Volts.of(3);

  public static double MIN_ANGLE = 0;
  public static double MAX_ANGLE = 270;

  public static double CURRENT_LIMT = 12;
  public static double GEARING = 1;
}
