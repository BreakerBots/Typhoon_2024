// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

/** Add your docs here. */
public class CANDeviceID {
    private String busName;
    private int deviceID;
    public CANDeviceID(String busName, int deviceID) {
        this.busName = busName;
        this.deviceID = deviceID;
    }

    public String getBusName() {
        return busName;
    }

    public int getDeviceID() {
        return deviceID;
    }
}
