package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class IntakeConstants {
  public static final Current CURRENT_LIMIT = Amps.of(30);
  public static final double GEARING = 2.0;
  public static final double INTAKE_POWER = 0.5;
}
