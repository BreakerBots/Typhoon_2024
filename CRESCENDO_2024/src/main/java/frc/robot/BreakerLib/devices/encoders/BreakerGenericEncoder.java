// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.encoders;


import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestable;

/** Add your docs here. */
public interface BreakerGenericEncoder extends BreakerSelfTestable, BreakerLoggable {
    /** Retruns a value in rotations [-0.5, 0.5] */
    public abstract double getEncoderAbsolutePosition();
    /** Retruns a value in rotations */
    public abstract double getEncoderRelativePosition();

    public abstract double getEncoderVelocity();

    public abstract void setEncoderPosition(double newPosition);

    public default void resetEncoderPosition() {
        setEncoderPosition(0.0);
    }

    @Override
    default void toLog(LogTable table) {
        table.put("AbsolutePositionRotations", getEncoderAbsolutePosition());
        table.put("RelativePositionRotations", getEncoderRelativePosition());
        table.put("VelocityRotationsPerSec", getEncoderVelocity());
        table.put("DeviceHealth", getHealth().toString());
    }
}
