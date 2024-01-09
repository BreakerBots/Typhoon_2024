// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;

/** Class which represents an analog HID trigger. */
public class BreakerAnalogTrigger {
    private GenericHID hid;
    private int port;
    private double deadband = 0.0;
    private boolean invert;

    /**
     * Construct uninverted trigger.
     * 
     * @param hid Controller.
     * @param analogTriggerAxisPort Axis port #.
     * @param invert Invert axis values.
     */
    public BreakerAnalogTrigger(GenericHID hid, int analogTriggerAxisPort) {
        this.hid = hid;
        port = analogTriggerAxisPort;
        invert = false;
    }

    /**
     * Construct trigger with invert.
     * 
     * @param hid Controller.
     * @param analogTriggerAxisPort Axis port #.
     * @param invert Invert axis values.
     */
    public BreakerAnalogTrigger(GenericHID hid, int analogTriggerAxisPort, boolean invert) {
        this.hid = hid;
        port = analogTriggerAxisPort;
    }

    /**
     * Deadband for trigger input. Equals 0 by default.
     * 
     * @param deadband Deadband value.
     */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    /** @return Trigger input without deadband. */
    public double getRaw() {
        return hid.getRawAxis(port);
    }

    /** @return Trigger input with deadband. */
    public double get() {
        return MathUtil.applyDeadband(getRaw(), deadband) * (invert ? -1 : 1);
    }
}
