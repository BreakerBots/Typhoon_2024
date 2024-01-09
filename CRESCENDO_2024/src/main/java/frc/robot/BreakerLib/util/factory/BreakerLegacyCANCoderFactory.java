        // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.factory;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;

/** Factory for producing CANcoders. */
@Deprecated
public class BreakerLegacyCANCoderFactory {

    /**
     * Creates CANCoder on default bus.
     * 
     * @param deviceID CAN ID
     * @param initializationStrategy Boot to zero or boot to absolute position.
     * @param absoluteSensorRange Unsigned 0 to 360 deg or signed +-180 deg.
     * @param absoluteOffsetDegrees Offset of encoder in degrees.
     * @param encoderDirection False = counterclockwise rotation is positive (facing towards LED).
     * 
     * @return CANCoder with desired settings.
     */
    public static WPI_CANCoder createCANCoder(int deviceID, SensorInitializationStrategy initializationStrategy,
            AbsoluteSensorRange absoluteSensorRange, double absoluteOffsetDegrees, boolean encoderDirection) {
        WPI_CANCoder encoder = new WPI_CANCoder(deviceID);
        configExistingCANCoder(encoder, initializationStrategy, absoluteSensorRange, absoluteOffsetDegrees,
                encoderDirection);
        return encoder;
    }

    /**
     * Creates CANCoder on specified bus.
     * 
     * @param deviceID CAN ID
     * @param busName CANivore device name/serial # or "rio" for RoboRIO bus.
     * 
     * @return CANCoder with desired settings.
     */
    public static WPI_CANCoder createCANCoder(int deviceID, String busName,
            SensorInitializationStrategy initializationStrategy, AbsoluteSensorRange absoluteSensorRange,
            double absoluteOffsetDegrees, boolean encoderDirection) {
        WPI_CANCoder encoder = new WPI_CANCoder(deviceID, busName);
        configExistingCANCoder(encoder, initializationStrategy, absoluteSensorRange, absoluteOffsetDegrees,
                encoderDirection);
        return encoder;
    }

    /**
     * Configure pre-existing CANCoder.
     * 
     * @param encoder CANCoder to config.
     * @param initializationStrategy Boot to zero or boot to absolute position.
     * @param absoluteSensorRange Unsigned 0 to 360 deg or signed +-180 deg.
     * @param absoluteOffsetDegrees Offset of encoder in degrees.
     * @param encoderDirection False = counterclockwise rotation is positive (facing towards LED).
     */
    public static void configExistingCANCoder(WPI_CANCoder encoder, SensorInitializationStrategy initializationStrategy,
            AbsoluteSensorRange absoluteSensorRange, double absoluteOffsetDegrees, boolean encoderDirection) {
        BreakerPhoenix5Util.checkError(encoder.configAbsoluteSensorRange(absoluteSensorRange),
                " CANCoder " + encoder.getDeviceID() + " ABS sensor range config fail ");
        BreakerPhoenix5Util.checkError(encoder.configSensorInitializationStrategy(initializationStrategy),
                " CANCoder " + encoder.getDeviceID() + " init stratagy config fail ");
        BreakerPhoenix5Util.checkError(encoder.configMagnetOffset(absoluteOffsetDegrees),
                " CANCoder " + encoder.getDeviceID() + " mag offset angle config fail ");
        BreakerPhoenix5Util.checkError(encoder.configSensorDirection(encoderDirection),
                " CANCoder " + encoder.getDeviceID() + " sensor direction config fail ");
    }
}
