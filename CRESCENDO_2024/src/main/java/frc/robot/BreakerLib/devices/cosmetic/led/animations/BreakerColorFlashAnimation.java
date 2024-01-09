// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led.animations;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class BreakerColorFlashAnimation extends BreakerColorSwitchAnimation {
    public BreakerColorFlashAnimation(int length, double flashTimeSeconds, Color flashColor) {
        super(length, flashTimeSeconds, flashColor, new Color(0, 0, 0));
    }
}
