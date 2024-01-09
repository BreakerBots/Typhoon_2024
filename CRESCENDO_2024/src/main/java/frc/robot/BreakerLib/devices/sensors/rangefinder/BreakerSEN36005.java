// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.rangefinder;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestable;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Add your docs here. */
public class BreakerSEN36005 extends TimeOfFlight implements BreakerSelfTestable {
    private String faultStr, deviceName;
    private DeviceHealth health;
    public BreakerSEN36005(int deviceID) {
        super(deviceID);
        faultStr = "";
        deviceName = "PWF_SEN36005_(" + deviceID + ")";
    }

    @Override
    public void runSelfTest() {
        Status stat = getStatus();
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        if (stat == Status.InternalError || stat == Status.HardwareFailure) {
            faultStr = stat.toString();
            health = DeviceHealth.INOPERABLE;
        }
        
    }

    @Override
    public DeviceHealth getHealth() {
        return health;
    }

    @Override
    public String getFaults() {
        return faultStr;
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    @Override
    public boolean hasFault() {
        return health != DeviceHealth.NOMINAL ;
    }

    @Override
    public void setDeviceName(String newName) {
        deviceName = newName;
        
    }
}
