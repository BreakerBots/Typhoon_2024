// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Class to represent gamepad thumsticks, stick clicks included. */
public class BreakerGamepadThumbstick extends BreakerAnalogThumbstick {
    private JoystickButton stickClick;

    /** */
    public BreakerGamepadThumbstick(GenericHID hid, int thumbstickButtonPort, int xAxisPort, boolean invertX,
            int yAxisPort, boolean invertY) {
        super(hid, xAxisPort, invertX, yAxisPort, invertY);
        stickClick = new JoystickButton(hid, thumbstickButtonPort);
    }

    public BreakerGamepadThumbstick(GenericHID hid, int thumbstickButtonPort, int xAxisPort, int yAxisPort) {
        super(hid, xAxisPort, yAxisPort);
        stickClick = new JoystickButton(hid, thumbstickButtonPort);
    }

    
    /** 
     * @return JoystickButton
     */
    public JoystickButton getJoystickButton() {
        return stickClick;
    }
}
