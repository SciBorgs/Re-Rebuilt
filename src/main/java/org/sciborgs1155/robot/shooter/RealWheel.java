package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.sciborgs1155.lib.FaultLogger.check;
import static org.sciborgs1155.lib.FaultLogger.register;
import static org.sciborgs1155.robot.shooter.ShooterConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.shooter.ShooterConstants.POSITION_FACTOR;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_FACTOR;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.ResetMode; // ResetMode must come from .spark
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealWheel implements WheelIO {
    private SparkFlex motor; // Motor Controller
    private RelativeEncoder encoder;
    private SparkFlexConfig config; // 

    public RealWheel(int id, boolean inverted) {
        motor = new SparkFlex(id, MotorType.kBrushless);
        encoder = motor.getEncoder();
        config = new SparkFlexConfig();

        check(
            motor,
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

        config.apply(config.idleMode(IdleMode.kCoast).smartCurrentLimit((int) CURRENT_LIMIT.in(Amps))); // Wheels just don't get power at kCoast

        config.apply(config.absoluteEncoder.inverted(inverted)); // Opposite direction

        config.apply(
            config
                .encoder
                .positionConversionFactor(POSITION_FACTOR.in(Radians))
                .velocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond))
                .uvwAverageDepth(16)
                .uvwMeasurementPeriod(32));

        config.apply(
            SparkUtils.getSignalsConfigurationFrameStrategy(
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.INTEGRATED),
                false));

        config.apply(config.inverted(inverted));

        check(
            motor,
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));

        register(motor);
    }

    @Override
    public void close() throws Exception {
        motor.close();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        check(motor);
    }

    @Override
    public double velocity() {
        return encoder.getVelocity();
    }
}
