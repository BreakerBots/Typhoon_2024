// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * ADXRS450 single axis gyro, included in the Kit of Parts. Only one can be used
 * on the robot.
 */
public class BreakerADXRS450 extends BreakerGenericDevice implements BreakerGenericGyro {

    private ADXRS450_Gyro gyro;

    /**
     * Creates an ADXRS450 gyro using the onboard CS0 SPI port.
     */
    public BreakerADXRS450() {
        gyro = new ADXRS450_Gyro();
        calibrate();
    }

    /**
     * Creates an ADXRS450 gyro with designated SPI port.
     * 
     * @param port SPI port for gyro.
     */
    public BreakerADXRS450(SPI.Port port) {
        gyro = new ADXRS450_Gyro(port);
        calibrate();
    }

    @Override
    public double getYaw() {
        return gyro.getAngle();
    }

    @Override
    public Rotation2d getYawRotation2d() {
        return gyro.getRotation2d();
    }

    /** Does nothing. */
    public void setYaw(double value) {
    }

    @Override
    public double getYawRate() {
        return gyro.getRate();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public double getRawYaw() {
        return getYaw();
    }

    @Override
    public double getRawYawRate() {
        return getYawRate();
    }

    @Override
    public void calibrate() {
        gyro.calibrate();
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        if (!gyro.isConnected()) {
            health = DeviceHealth.INOPERABLE;
            faultStr = " GYRO_NOT_CONNECTED ";
          }
        
    }

    @Override
    public void close() throws Exception {
        gyro.close();
        
    }
}
