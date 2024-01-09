// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.devices.BreakerGenericLoopedDevice;
import frc.robot.BreakerLib.devices.cosmetic.led.animations.BreakerAnimation;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotOperatingMode;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;

/** CTRE CANdle LED controller. */
public class BreakerCANdle extends BreakerGenericLoopedDevice implements BreakerGenericLED {

    private CANdle candle;
    private boolean isOn = false, isActiveBAnim;

    /**
     * Creates a BreakerCANdle on the default CANBus.
     * 
     * @param canID  CAN ID for CANdle.
     */
    public BreakerCANdle(int canID) {
        candle = new CANdle(canID);
        deviceName = " CANdle_LED_Controller (" + canID + ") ";
    }

    /**
     * Creates a BreakerCANdle on named CANBus.
     * 
     * @param canID   CAN ID for CANdle.
     * @param busName CAN bus name.
     */
    public BreakerCANdle(int canID, String busName) {
        candle = new CANdle(canID, busName);
        deviceName = " CANdle_LED_Controller (" + canID + ") ";
    }

    /**
     * Applies all settings in provided configuration.
     * 
     * @param config Configuration to apply.
     */
    public void applyConfig(CANdleConfiguration config) {
        candle.configAllSettings(config);
    }

    /**
     * Sets LEDs to play CTRE LED animation.
     * 
     * @param animation CANdle animation.
     */
    public void setCTREAnimation(Animation animation) {
        candle.animate(animation);
    }

    @Override
    public void setAllLEDs(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    @Override
    public void setLEDsInRange(int r, int g, int b, int startIndex, int endIndex) {
        candle.setLEDs(r, g, b, 0, startIndex, endIndex - startIndex);

    }

    @Override
    public void setLED(int r, int g, int b, int index) {
        candle.setLEDs(r, g, b, 0, index, 1);

    }

    @Override
    public void setAnimation(BreakerAnimation state) {

    }

    @Override
    public void setAnimation(int startIndex, BreakerAnimation animation) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setAnimation(int startIndex, int endIndex, BreakerAnimation animation) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setRobotModeDefaultState(RobotOperatingMode mode, BreakerAnimation animation) {
        // TODO Auto-generated method stub

    }

    @Override
    public void clearToModeDefault() {
        // TODO Auto-generated method stub

    }

    @Override
    public void clearActiveAnimation() {
        // TODO Auto-generated method stub

    }

    @Override
    public void setOn() {
        isOn = true;
    }

    @Override
    public void setOff() {
        isOn = false;
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public boolean isInModeDefault() {
        return isActiveBAnim;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        CANdleFaults faultsC = new CANdleFaults();
        candle.getFaults(faultsC);
        if (faultsC.hasAnyFault()) {
            Pair<DeviceHealth, String> faultData = BreakerPhoenix5Util.getCANdleHealthAndFaults(faultsC);
            faultStr = faultData.getSecond();
            health = faultData.getFirst();
        } else {
            health = DeviceHealth.NOMINAL;
        }

    }
}
