// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;

/** Single-axis gyro connected to analog ports of RoboRIO. Gyro is initialized on construction. */
public class BreakerAnalogGyro implements BreakerGenericGyro {

    private AnalogGyro gyro;

    /**
     * Creates a BreakerAnalogGyro.
     * 
     * @param port Analog port on RoboRIO. Must be port 0 or 1.
     */
    public BreakerAnalogGyro(int port) {
        gyro = new AnalogGyro(port);
        gyro.initGyro();
        calibrate();
    }

    /**
     * Creates a BreakerAnalogGyro.
     * 
     * @param port   Analog port on RoboRIO. Must be port 0 or 1.
     * @param center Uncalibrated value to use as center value.
     * @param offset Gyro offset.
     */
    public BreakerAnalogGyro(int port, int center, int offset) {
        gyro = new AnalogGyro(port, center, offset);
        gyro.initGyro();
        calibrate();
    }

    /**
     * Sets gyro sensitivity, in volts/degree/sec. This value will differ between
     * analog gyros.
     * 
     * @param voltsPerDegreePerSecond Sensitivity for given gyro.
     */
    public void setSensitivity(double voltsPerDegreePerSecond) {
        gyro.setSensitivity(voltsPerDegreePerSecond);
    }

    /**
     * Sets gyro deadband. Increase in deadband voltage lowers drift at the cost of
     * accuracy.
     * 
     * @param volts Volt deadband.
     */
    public void setDeadband(double volts) {
        gyro.setDeadband(volts);
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
    public void close() throws Exception {
        gyro.close();
    }
}
