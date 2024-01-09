// package frc.robot.BreakerLib.devices.motors.binary;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.binary;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

/** Falcon motor with simple on/off controls */
public class BreakerBinarySparkMax extends BreakerGenericBinaryMotor {

    private CANSparkMax motor;
    private double output;

    /**
     * Create a new BinaryCTREMotor that switches between 100% forward output and 0%
     * output.
     * 
     * @param motor CTRE motor controller.
     */
    public BreakerBinarySparkMax(CANSparkMax motor) {
        this.motor = motor;
        this.output = 1.0;
        deviceName = " Binary_Motor (" + motor.getDeviceId() + ") ";

    }

    /**
     * Create a new BinaryCTREMotor that switches between given output % and 0%
     * output.
     * 
     * @param motor  CTRE motor controller.
     * @param output Percent output between -1 and 1.
     */
    public BreakerBinarySparkMax(CANSparkMax motor, double output) {
        this.motor = motor;
        this.output = output;
        deviceName = " Binary_Motor (" + motor.getDeviceId() + ") ";
    }

    @Override
    /** Sets motor to designated percent output. */
    public void start() {
        motor.set(output);
    }

    @Override
    /** Sets motor to 0% output (stopped) */
    public void stop() {
        motor.set(0);
    }

    @Override
    /** Checks if motor is running or not. */
    public boolean isActive() {
        return (motor.get() != 0);
    }

    /** @return Base CANSparkMax motor controller. */
    public CANSparkMax getMotor() {
        return motor;
    }

    @Override
    public void toggle() {
        if (isActive()) {
            stop();
        } else {
            start();
        }
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        short faults = motor.getFaults();
        if (faults != 0) {
            Pair<DeviceHealth, String> pair = BreakerREVUtil.getSparkMaxHealthAndFaults(faults);
            faultStr = pair.getSecond();
            health = pair.getFirst();
        }
    }

}
