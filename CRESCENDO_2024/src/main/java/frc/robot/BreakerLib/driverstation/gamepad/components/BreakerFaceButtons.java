// Copyright (c) FIRST and other WPILib contributors.BreakerFaceButtons
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A class that represents the face button set common to most gamepad
 * controllers,
 * ex. ABXY buttons of Xbox controllers or the shape buttons of PlayStation
 * controllers.
 */
public class BreakerFaceButtons {
    private JoystickButton topFaceButton, leftFaceButton, rightFaceButton, bottomActonButton;

    /** Constructs set of face button based on given button ports and HID device. */
    public BreakerFaceButtons(GenericHID hid, int topFaceButtonPort, int leftFaceButtonPort,
            int rightFaceButtonPort, int bottomFaceButtonPort) {
        topFaceButton = new JoystickButton(hid, topFaceButtonPort);
        leftFaceButton = new JoystickButton(hid, leftFaceButtonPort);
        rightFaceButton = new JoystickButton(hid, rightFaceButtonPort);
        bottomActonButton = new JoystickButton(hid, bottomFaceButtonPort);
    }

    /** @return Bottom face button (ex. A on Xbox). */
    public JoystickButton getBottomFaceButton() {
        return bottomActonButton;
    }

    /** @return Left face button (ex. X on Xbox). */
    public JoystickButton getLeftFaceButton() {
        return leftFaceButton;
    }

    /** @return Right face button (ex. B on Xbox). */
    public JoystickButton getRightFaceButton() {
        return rightFaceButton;
    }

    /** @return Top face button (ex. Y on Xbox). */
    public JoystickButton getTopFaceButton() {
        return topFaceButton;
    }
}
