package org.sciborgs1155.robot.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class TurretConstants {
  public static double kP = 9;
  public static double kI = 0;
  public static double kD = 0.1;
  public static double kS = 0;
  public static double kG = 0;
  public static double kA = 0.1;
  public static double kV = 0;

  public static DCMotor GEARBOX = DCMotor.getNEO(2);

  // public static double TOLERANCE = 3;
  public static double TOLERANCE = 0.1;
  public static Velocity<VoltageUnit> QUASISTATIC_VOLTAGE = Volts.per(Second).of(2);
  public static Voltage DYNAMIC_VOLTAGE = Volts.of(3);

  public static double MIN_ANGLE = 0;
  public static double MAX_ANGLE = 270;

  public static Current CURRENT_LIMIT = Amps.of(60);
  public static double GEARING = 686 / 15;

  public static double MOI = 0.0872;

  public static final int TURRET_GEARING = 84;
  public static final int ENCODER_A_GEARING = 12;
  public static final int ENCODER_B_GEARING = 13;

  public static final Angle CRT_MATCH_TOLERANCE = Degrees.of(1);
}
