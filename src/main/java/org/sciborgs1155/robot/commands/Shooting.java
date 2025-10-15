package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.drive.DriveConstants.RADIUS;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_VELOCITY;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.shooter.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import monologue.Annotations.IgnoreLogged;
import monologue.Logged;

public class Shooting implements Logged {
    // is entry internal storage or constant info to obtain?
    public static final DoubleEntry siggysConstant = Tuning.entry("/Robot/Siggy's Constant", 4.42);

    public static final Distance MAX_DISTANCE = Meters.of(5.0);

    // what is InterpolatingDoubleTreeMap?
    private static final InterpolatingDoubleTreeMap shotVelocityLookup = new InterpolatingDoubleTreeMap();

    @IgnoreLogged
    private final Shooter shooter;

    public Shooting(Shooter shooter) {
        this.shooter = shooter;

        shotVelocityLookup.put(0.0, 300.0);
        shotVelocityLookup.put(1.0, 450.0);
        shotVelocityLookup.put(4.0, MAX_VELOCITY.in(RadiansPerSecond));
    }

    /**
     * Runs the shooter before feeding it the note.
     *
     * @param desiredVelocity The velocity in radians per second to shoot at.
     * @return The command to shoot at the desired velocity.
     */
    public Command shoot(AngularVelocity desiredVelocity) {
        return shoot(() -> desiredVelocity.in(RadiansPerSecond), () -> true);
    }

    /**
     * Runs shooter to desired velocity, runs feeder once it reaches its velocity
     * and shootCondition
     * is true.
     *
     * @param desiredVelocity Target velocity for the flywheel.
     * @param shootCondition  Condition after which the feeder will run.
     */

    public Command shoot(DoubleSupplier desiredVelocity, BooleanSupplier shootCondition) {
        return Commands.waitUntil(
            () -> shooter.atVelocity(desiredVelocity.getAsDouble()) && shootCondition.getAsBoolean())
            .deadlineFor(shooter.runShooter(desiredVelocity));
    }

    /**
     * Converts between flywheel speed and note speed
     *
     * @param flywheelSpeed Flywheel speed in radians per second
     * @return Note speed in meters per second
     */

    public static double flywheelToNoteSpeed(double flywheelSpeed) {
        return flywheelSpeed * RADIUS.in(Meters) / siggysConstant.get();
    }
}
