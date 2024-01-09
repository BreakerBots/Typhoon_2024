
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.trinary;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

/** Falcon motor with simple forward/reverse/off controls */
public class BreakerTrinarySparkMax extends BreakerGenericTrinaryMotor {

    private CANSparkMax motor;
    private double forwardOut, reverseOut;

    /**
     * Create a new TrinaryCTREMotor with 100% forward input and -100% reverse input.
     * 
     * @param motor CTRE motor controller.
     */
    public BreakerTrinarySparkMax(CANSparkMax motor) {
        new BreakerTrinarySparkMax(motor, 1, -1);
    }

    /**
     * Create a new TrinaryCTREMotor that has given output % forward, negative given output % reverse.
     * 
     * @param motor  CTRE motor controller.
     * @param output Percent output between -1 and 1.
     */
    public BreakerTrinarySparkMax(CANSparkMax motor, double output) {
        new BreakerTrinarySparkMax(motor, output, -output);
    }

    /**
     * Create a new TrinaryCTREMotor that has separate forward output % and reverse output %
     * 
     * @param motor CTRE motor controller.
     * @param forwardOut Forward % output between -1 and 1.
     * @param reverseOut Reverse % output between -1 and 1.
     */
    public BreakerTrinarySparkMax(CANSparkMax motor, double forwardOut, double reverseOut) {
        this.motor = motor;
        this.forwardOut = forwardOut;
        this.reverseOut = reverseOut;
        deviceName = " Trinary_Motor (" + motor.getDeviceId() + ") ";
    }

    @Override
    /** Sets motor to designated forward percent output. */
    public void forward() {
        motor.set(forwardOut);
    }

    @Override
    /** Sets motor to designated reverse percent output. */
    public void reverse() {
        motor.set(reverseOut);
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
