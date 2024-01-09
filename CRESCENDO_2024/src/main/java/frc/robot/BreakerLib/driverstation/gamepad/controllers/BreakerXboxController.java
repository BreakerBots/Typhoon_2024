// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Wrapper for Xbox controller inputs. */
public class BreakerXboxController extends BreakerGenericGamepad {

    // Xbox digital button ports
    public static final int A_PORT = 1;
    public static final int B_PORT = 2;
    public static final int X_PORT = 3;
    public static final int Y_PORT = 4;
    public static final int L_BUMP_PORT = 5;
    public static final int R_BUMP_PORT = 6;
    public static final int BACK_PORT = 7;
    public static final int START_PORT = 8;
    public static final int L_STICK_PRESS_PORT = 9;
    public static final int R_STICK_PRESS_PORT = 10;
    // Xbox axis ports
    public static final int LEFT_X_PORT = 0;
    public static final int LEFT_Y_PORT = 1;
    public static final int L_TRIGGER_PORT = 2;
    public static final int R_TRIGGER_PORT = 3;
    public static final int RIGHT_X_PORT = 4;
    public static final int RIGHT_Y_PORT = 5;
    // Xbox D-Pad angle constants
    public static final int DPAD_UP_ANG = 0;
    public static final int DPAD_TOP_RIGHT_ANG = 45;
    public static final int DPAD_RIGHT_ANG = 90;
    public static final int DPAD_BOTTOM_RIGHT_ANG = 135;
    public static final int DPAD_DOWN_ANG = 180;
    public static final int DPAD_BOTTOM_LEFT_ANG = 225;
    public static final int DPAD_LEFT_ANG = 270;
    public static final int DPAD_TOP_LEFT_ANG = 315;

    private JoystickButton startButton;
    private JoystickButton backButton;

    /** Xbox controller based on the {@link BreakerGenericGamepad}. 
     * 
     * @param xboxPortNum Xbox port number.
    */
    public BreakerXboxController(int xboxPortNum) {
        super(
            new XboxController(xboxPortNum),
            Y_PORT, X_PORT, B_PORT, A_PORT, 
            L_STICK_PRESS_PORT, XboxController.Axis.kLeftX.value, 
            true, XboxController.Axis.kLeftY.value, 
            true, R_STICK_PRESS_PORT, 
            XboxController.Axis.kRightX.value, true, 
            XboxController.Axis.kRightY.value, true, 
            L_TRIGGER_PORT, R_TRIGGER_PORT, L_BUMP_PORT, R_BUMP_PORT
        );
        startButton = new JoystickButton(hid, START_PORT);
        backButton = new JoystickButton(hid, BACK_PORT);
    }

    /** @return {@link JoystickButton} for the back button (Two squares). */
    public JoystickButton getBackButton() {
        return backButton;
    }

    /** @return {@link JoystickButton} for the start button (Three lines). */
    public JoystickButton getStartButton() {
        return startButton;
    }

    /** @return A button. */
    public JoystickButton getButtonA() {
        return faceButtons.getBottomFaceButton();
    }

    /** @return B button. */
    public JoystickButton getButtonB() {
        return faceButtons.getRightFaceButton();
    }

    /** @return X button. */
    public JoystickButton getButtonX() {
        return faceButtons.getLeftFaceButton();
    }

    /** @return Y button. */
    public JoystickButton getButtonY() {
        return faceButtons.getTopFaceButton();
    }
}
