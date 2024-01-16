// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.ejml.equation.Function;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public interface BreakerSwerveAzimuthEncoder {

    /** @return Relative anglular position in rotations, (0.5 -> 0.6) */
    public abstract double getRelative();

    /** @return Absolute anglular position in rotations [-0.5, 0.5]. */
    public abstract double getAbsolute();

    /**
     * Configures the encoder.
     * 
     * @param clockwisePositive Sets whether roation that would normaly be read as positive should be negative
     * @param offset            Angle offset in degrees.
     */
    public abstract void config(boolean invertEncoder, double offset);

    public abstract Pair<DeviceHealth, String> getFaultData();

    public abstract Object getBaseEncoder();

    public abstract void setStatusUpdatePeriod(double period);

    //public abstract BreakerSwerveAzimuthEncoderSimIO getSimIO();

    public static interface BreakerSwerveAzimuthEncoderSimIO {

        public abstract void setSimAngle(double angle);

        public abstract void setSimAngleVel(double angleVel);

        public abstract void setSimSupplyVoltage(double supplyVoltage);

        public abstract void setInverted(boolean isInverted);
        
    };


}