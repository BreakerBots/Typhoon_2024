        // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.factory;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** Factory for producing CANcoders. */
public class BreakerCANCoderFactory {

    /**
     */
    public static CANcoder createCANCoder(int deviceID, AbsoluteSensorRangeValue absoluteSensorRange, double absoluteOffsetRotations, SensorDirectionValue encoderDirection) {
        
        return createCANCoder(deviceID, "rio", absoluteSensorRange, absoluteOffsetRotations, encoderDirection);
    }

    /**
     */
    public static CANcoder createCANCoder(int deviceID, String busName,
        AbsoluteSensorRangeValue absoluteSensorRange, double absoluteOffsetRotations, SensorDirectionValue encoderDirection) {
        CANcoder encoder = new CANcoder(deviceID, busName);
        configExistingCANCoder(encoder, absoluteSensorRange, absoluteOffsetRotations, encoderDirection);
        return encoder;
    }

    /**
     */
    public static void configExistingCANCoder(CANcoder encoder, AbsoluteSensorRangeValue absoluteSensorRange, double absoluteOffsetRotations, SensorDirectionValue encoderDirection) {
        CANcoderConfiguration config =  new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = absoluteSensorRange;
        config.MagnetSensor.MagnetOffset = absoluteOffsetRotations;
        config.MagnetSensor.SensorDirection = encoderDirection;
        StatusCode cod = encoder.getConfigurator().apply(config);    
        //BreakerPhoenix6Util.checkStatusCode(encoder.getConfigurator().apply(config),  " CANcoder " + encoder.getDeviceID() + " general config fail ");
    }
}
