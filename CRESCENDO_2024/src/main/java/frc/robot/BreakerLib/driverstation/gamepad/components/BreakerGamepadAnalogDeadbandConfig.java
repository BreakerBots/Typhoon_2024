// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

/** Configuration for all axis deadbands for a given gamepad. */
public class BreakerGamepadAnalogDeadbandConfig {

    private double leftStick = 0.0, rightStick = 0.0, leftTriggerAxis = 0.0, rightTriggerAxis = 0.0;

    /**
     * Constructs deadband config for both sticks and triggers
     * 
     * @param leftX Left stick X-axis deadband.
     * @param leftY Left stick Y-axis deadband.
     * @param rightX Right stick X-axis deadband.
     * @param rightY Right stick Y-axis deadband.
     * @param leftTriggerAxis Left triggert deadband.
     * @param rightTriggerAxis Right trigger deadband.
     */
    public BreakerGamepadAnalogDeadbandConfig(double leftStick, double rightStick, double leftTriggerAxis, double rightTriggerAxis) {
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.leftTriggerAxis = leftTriggerAxis;
        this.rightTriggerAxis = rightTriggerAxis;
    }

    /**
     * Constructs deadband config for both analog sticks.
     * 
     * @param leftX Left stick X-axis deadband.
     * @param leftY Left stick Y-axis deadband.
     * @param rightX Right stick X-axis deadband.
     * @param rightY Right stick Y-axis deadband.
     */
    public BreakerGamepadAnalogDeadbandConfig(double leftStick, double rightStick) {
        this(leftStick, rightStick, 0.0, 0.0);
    }

    /** Deadbands are all set to 0. */
    public BreakerGamepadAnalogDeadbandConfig() {
        this(0.0, 0.0);
    }
    
    /** @return Left trigger deadband. */
    public double getLeftTriggerAxis() {
        return leftTriggerAxis;
    }

    /** @return Left stick hypot deadband. */
    public double getLeftStick() {
        return leftStick;
    }

    /** @return Right trigger deadband. */
    public double getRightTriggerAxis() {
        return rightTriggerAxis;
    }

    /** @return Right stick hypot deadband. */
    public double getRightStick() {
        return rightStick;
    }
}
