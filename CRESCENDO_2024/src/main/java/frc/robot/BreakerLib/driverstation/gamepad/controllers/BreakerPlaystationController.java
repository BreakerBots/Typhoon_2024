// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.controllers;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Playstation DualShock/DualSense controller wrapper. */
public class BreakerPlaystationController extends BreakerGenericGamepad {
    private JoystickButton share, options, touchpad, ps;

    /** PlayStation controller based on the {@link BreakerGenericGamepad}. 
     * 
     * @param xboxPortNum PlayStation port number.
    */
    public BreakerPlaystationController(int controllerPortNum) {
        super(new PS4Controller(controllerPortNum), PS4Controller.Button.kTriangle.value,
                PS4Controller.Button.kSquare.value, PS4Controller.Button.kCircle.value,
                PS4Controller.Button.kCross.value,
                PS4Controller.Button.kL3.value, PS4Controller.Axis.kLeftX.value, false, PS4Controller.Axis.kLeftY.value,
                false, PS4Controller.Button.kR3.value, PS4Controller.Axis.kRightX.value, false,
                PS4Controller.Axis.kRightY.value, false, PS4Controller.Axis.kL2.value, PS4Controller.Axis.kR2.value,
                PS4Controller.Button.kL1.value,
                PS4Controller.Button.kL1.value);
        share = new JoystickButton(hid, PS4Controller.Button.kShare.value);
        options = new JoystickButton(hid, PS4Controller.Button.kOptions.value);
        touchpad = new JoystickButton(hid, PS4Controller.Button.kTouchpad.value);
        ps = new JoystickButton(hid, PS4Controller.Button.kPS.value);

    }

    /** @return Triangle button. */
    public JoystickButton getTriangleButton() {
        return getFaceButtons().getTopFaceButton();
    }

    /** @return Square button. */
    public JoystickButton getSquareButton() {
        return getFaceButtons().getLeftFaceButton();
    }

    /** @return Circle button. */
    public JoystickButton getCircleButton() {
        return getFaceButtons().getRightFaceButton();
    }

    /** @return Cross button. */
    public JoystickButton getCrossButton() {
        return getFaceButtons().getBottomFaceButton();
    }

    /** @return Share button. */
    public JoystickButton getShareButton() {
        return share;
    }

    /** @return Option button. */
    public JoystickButton getOptionsButton() {
        return options;
    }

    /** @return PS button. */
    public JoystickButton getPlayStationButton() {
        return ps;
    }

    /** @return Touchpad click. */
    public JoystickButton getTouchpadButton() {
        return touchpad;
    }

}
