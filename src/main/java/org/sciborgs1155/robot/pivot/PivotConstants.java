package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class PivotConstants {
    // What now?
    public static final double MOTOR_GEARING = 12.0 / 64.0 * 20.0 / 70.0 * 36.0 / 56.0 * 16.0 / 54.0;

    public static final Mass MASS = Kilograms.of(1);
    public static final Distance LENGTH = Inches.of(16);

    public static final Angle STARTING_ANGLE = Degrees.of(63.3);
    public static final Angle MIN_ANGLE = Degrees.of(-45.7);
    public static final Angle MAX_ANGLE = STARTING_ANGLE.minus(Degrees.of(4.1));

    // PivotVisualizer
    public static final Translation3d AXLE_FROM_CHASSIS = new Translation3d(Inches.of(-10.465), Inches.zero(),
            Inches.of(25)); // can't another type be used instead of translation3d?
}
