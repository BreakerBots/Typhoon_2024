// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led.animations;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public interface BreakerAnimation {
    public abstract void start();
    public abstract void stop();
    public abstract boolean isActive();
    public abstract AddressableLEDBuffer getBuffer();
    public default Color getColorAtIndex(int index) {
        return getBuffer().getLED(index);
    }
    public default Color8Bit getColor8BitAtIndex(int index) {
        return new Color8Bit(getColorAtIndex(index));
    }
    public abstract int getLength();

}
