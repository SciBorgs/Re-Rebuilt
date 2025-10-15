package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*; // Amps

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterConstants {
    public static final double GEARING = 1;

    public static final Current CURRENT_LIMIT = Amps.of(45); // Max voltage a motor can have
    public static final Angle POSITION_FACTOR = Rotations.one();
    public static final AngularVelocity VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

    // Shooter.java
    public static final AngularVelocity DEFAULT_VELOCITY = RadiansPerSecond.of(550);
    public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(630);
    
    public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(5);
    public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(300);

    public static final double kP = 0.03;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final class Top {
        public static final double kS = 0;
        public static final double kV = 0.016896;
        public static final double kA = 0.0031483;
    } // Specifically what do these values represent? I know they involve feedfoward

    public static final class Bottom {
        public static final double kS = 0.038488;
        public static final double kV = 0.016981;
        public static final double kA = 0.0021296;
    }
}
