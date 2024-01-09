// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Class to represent directional pads common on most controllers. */
public class BreakerDPad {

    /** Degree values for all inputs on a D-Pad. */
    public static final class DefaultButtonAngles {
        public static final int UP = 0;
        public static final int TOP_RIGHT = 45;
        public static final int RIGHT = 90;
        public static final int BOTTOM_RIGHT = 135;
        public static final int DOWN = 180;
        public static final int BOTTOM_LEFT = 225;
        public static final int LEFT = 270;
        public static final int TOP_LEFT = 315;
    }

    private POVButton dPadUp;
    private POVButton dPadTopRight;
    private POVButton dPadDown;
    private POVButton dPadBottomRight;
    private POVButton dPadLeft;
    private POVButton dPadBottomLeft;
    private POVButton dPadRight;
    private POVButton dPadTopLeft;

    /** D-pad configured with default angles. */
    public BreakerDPad(GenericHID hid) {
        this(
                hid,
                DefaultButtonAngles.UP,
                DefaultButtonAngles.TOP_RIGHT,
                DefaultButtonAngles.RIGHT,
                DefaultButtonAngles.BOTTOM_RIGHT,
                DefaultButtonAngles.DOWN,
                DefaultButtonAngles.BOTTOM_LEFT,
                DefaultButtonAngles.LEFT,
                DefaultButtonAngles.TOP_LEFT);
    }

    /**
     * Full constructor for BreakerDPad.
     * 
     * @param hid Controller to use.
     * @param upButtonAng
     * @param topRightButtonAng
     * @param rightButtonAng
     * @param bottomRightButtonAng
     * @param downButtonAng
     * @param bottomLeftButtonAng
     * @param leftButtonAng
     * @param topLeftButtonAng
     */
    public BreakerDPad(
            GenericHID hid,
            int upButtonAng,
            int topRightButtonAng,
            int rightButtonAng,
            int bottomRightButtonAng,
            int downButtonAng,
            int bottomLeftButtonAng,
            int leftButtonAng,
            int topLeftButtonAng) {
        dPadUp = new POVButton(hid, upButtonAng);
        dPadTopRight = new POVButton(hid, topRightButtonAng);
        dPadDown = new POVButton(hid, downButtonAng);
        dPadBottomRight = new POVButton(hid, bottomRightButtonAng);
        dPadLeft = new POVButton(hid, leftButtonAng);
        dPadBottomLeft = new POVButton(hid, bottomLeftButtonAng);
        dPadRight = new POVButton(hid, rightButtonAng);
        dPadTopLeft = new POVButton(hid, topLeftButtonAng);
    }

    /** @return DPad down input. */
    public POVButton getDown() {
        return dPadDown;
    }

    /** @return DPad left input. */
    public POVButton getLeft() {
        return dPadLeft;
    }

    /** @return DPad right input. */
    public POVButton getRight() {
        return dPadRight;
    }

    /** @return DPad up input. */
    public POVButton getUp() {
        return dPadUp;
    }

    /** @return DPad up-left input. */
    public POVButton getTopLeft() {
        return dPadTopLeft;
    }

    /** @return DPad up-right input. */
    public POVButton getTopRight() {
        return dPadTopRight;
    }

    /** @return DPad down-left input. */
    public POVButton getBottomLeft() {
        return dPadBottomLeft;
    }

    /** @return DPad down-right input. */
    public POVButton getBottomRight() {
        return dPadBottomRight;
    }

}
