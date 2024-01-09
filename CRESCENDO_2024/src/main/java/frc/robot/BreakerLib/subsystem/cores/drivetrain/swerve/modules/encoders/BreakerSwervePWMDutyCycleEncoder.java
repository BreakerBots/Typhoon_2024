// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * PWM-controlled duty cycle encoders.
 * 
 * Devices to use include the following:
 * - CTRE Mag Encoder (connected to Rio)
 * - Rev Through Bore
 * - US Digital MA3 and other simple PWM encoders.
 */
public class BreakerSwervePWMDutyCycleEncoder implements BreakerSwerveAzimuthEncoder {

    private DutyCycleEncoder dcEncoder;
    private double offset;
    public BreakerSwervePWMDutyCycleEncoder(int channel, int connectedFrequencyThreshold, double dutyCycleMin, double dutyCycleMax) {
        dcEncoder = new DutyCycleEncoder(channel);
        dcEncoder.setDutyCycleRange(dutyCycleMin, dutyCycleMax);
        dcEncoder.setDistancePerRotation(1.0);
        dcEncoder.setConnectedFrequencyThreshold(connectedFrequencyThreshold);
    }

    @Override
    public double getRelative() {
        return dcEncoder.getDistance() + offset;
    }

    @Override
    public double getAbsolute() {
        return MathUtil.inputModulus(dcEncoder.getDistance() + offset, -0.5, 0.5);
    }

    @Override
    public void config(boolean invertEncoder, double offset) {
        dcEncoder.setDistancePerRotation(invertEncoder ? -1.0 : 1.0 );
        this.offset = offset;
    }

    @Override
    public Pair<DeviceHealth, String> getFaultData() {
        if (!dcEncoder.isConnected()) {
            return new Pair<DeviceHealth,String>(DeviceHealth.INOPERABLE, " pwm_encoder_disconnected ");
        }
       return new Pair<DeviceHealth,String>(DeviceHealth.NOMINAL, "");
    }

    @Override
    public Object getBaseEncoder() {
        return dcEncoder;
    }
}
