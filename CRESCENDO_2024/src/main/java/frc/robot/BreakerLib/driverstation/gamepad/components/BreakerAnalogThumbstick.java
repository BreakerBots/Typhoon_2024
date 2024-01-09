// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;

/** Class which represents an analog HID thumbstick. */
public class BreakerAnalogThumbstick {
    private GenericHID hid;
    private int xAxisPort, yAxisPort;
    private boolean invertX, invertY;
    private double deadband = 0.0;
    

    /**
     * Constructs an analog thumbstick with desired inverts.
     * 
     * @param hid Controller.
     * @param xAxisPort X-axis port #.
     * @param yAxisPort Y-axis port #.
     */
    public BreakerAnalogThumbstick(GenericHID hid, int xAxisPort, int yAxisPort) {
        this(hid, xAxisPort, false, yAxisPort, false);
    }

    /**
     * Constructs an analog thumbstick with desired inverts.
     * 
     * @param hid Controller.
     * @param xAxisPort X-axis port #.
     * @param invertX Invert X-axis.
     * @param yAxisPort Y-axis port #.
     * @param invertY Invert Y-axis.
     */
    public BreakerAnalogThumbstick(GenericHID hid, int xAxisPort, boolean invertX, int yAxisPort, boolean invertY) {
        this.hid = hid;
        this.xAxisPort = xAxisPort;
        this.yAxisPort = yAxisPort;
        this.invertX = invertX;
        this.invertY = invertY;
        deadband = 0.0;
    }

    /** Set stick deadbands.
     * 
     * @param deadband Magnitude deadband for stick reading vector.
     */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    /** @return Raw X-axis value. */
    public double getRawX() {
        return hid.getRawAxis(xAxisPort);
    }

    /** @return Raw Y-axis value. */
    public double getRawY() {
        return hid.getRawAxis(yAxisPort);
    }

    public BreakerVector2 getRawVector() {
        return new BreakerVector2(getRawX(), getRawY());
    }

    public BreakerVector2 getVector() {
        BreakerVector2 vec = new BreakerVector2(getRawX() * (invertX ? -1 : 1), getRawY() * (invertY ? -1 : 1));
        return new BreakerVector2(vec.getVectorRotation(), MathUtil.applyDeadband(vec.getMagnitude(), deadband));
    }

    /** @return X-axis value. */
    public double getX() {
        return getVector().getX();
    }

    /** @return Y-axis value. */
    public double getY() {
        return getVector().getY();
    }

    /** @return If stick inputs outside of the deadband are detected. */
    public boolean isActive() {
        return (getY() != 0 || getX() !=0);
    }
}
