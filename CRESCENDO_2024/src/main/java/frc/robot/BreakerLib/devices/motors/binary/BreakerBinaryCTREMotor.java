// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.binary;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;

/** Falcon motor with simple on/off controls */
public class BreakerBinaryCTREMotor extends BreakerGenericBinaryMotor {

    private BaseMotorController motor;
    private double output;

    /**
     * Create a new BinaryTalon that switches between 100% forward output and 0%
     * output.
     * 
     * @param motor CTRE motor controller.
     */
    public BreakerBinaryCTREMotor(BaseMotorController motor) {
        this.motor = motor;
        this.output = 1.0;
        deviceName = " Binary_Motor (" + motor.getDeviceID() + ") ";

    }

    /**
     * Create a new BinaryTalon that switches between given output % and 0%
     * output.
     * 
     * @param motor  CTRE motor controller.
     * @param output Percent output between -1 and 1.
     */
    public BreakerBinaryCTREMotor(BaseMotorController motor, double output) {
        this.motor = motor;
        this.output = output;
        deviceName = " Binary_Motor (" + motor.getDeviceID() + ") ";
    }

    @Override
    /** Sets motor to designated percent output. */
    public void start() {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    /** Sets motor to 0% output (stopped) */
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    /** Checks if motor is running or not. */
    public boolean isActive() {
        return (motor.getMotorOutputPercent() != 0);
    }

    /** @return Base CTRE motor controller. */
    public BaseMotorController getMotor() {
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
        Pair<DeviceHealth, String> faultPair = BreakerPhoenix5Util.checkMotorFaultsAndConnection(motor);
        if (faultPair.getFirst() != DeviceHealth.NOMINAL) {
            faultStr = faultPair.getSecond();
            health = faultPair.getFirst();
        }
    }

}
