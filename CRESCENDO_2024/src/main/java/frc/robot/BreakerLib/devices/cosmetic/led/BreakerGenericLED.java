// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.BreakerLib.devices.cosmetic.led.animations.BreakerAnimation;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotOperatingMode;

/** Driver for all LED controllers in BreakerLib. */
public interface BreakerGenericLED {
    public static final Color BREAKER_RED = new Color(new Color8Bit(196, 30, 58));
    public static final Color BREAKER_GOLD = new Color(new Color8Bit(253, 181, 21));

    public abstract void setAllLEDs(int r, int g, int b);

    /**
     * Set LEDs to one 12-bit color (converted to 8-bit).
     * 
     * @param ledColor LED color for all lights.
     */
    public default void setAllLEDs(Color ledColor) {
        setAllLEDs(new Color8Bit(ledColor));
    }

    /**
     * Sets LEDs to one 8-bit color.
     * 
     * @param ledColor LED color for all lights.
     */
    public default void setAllLEDs(Color8Bit ledColor) {
        setAllLEDs(ledColor.red, ledColor.green, ledColor.blue);
    }

    /**
     * Sets selected LEDs to given color.
     * 
     * @param r          Red value.
     * @param g          Green value.
     * @param b          Blue value.
     * @param startIndex Start of LED selection.
     * @param endIndex   End of LED selection.
     */
    public abstract void setLEDsInRange(int r, int g, int b, int startIndex, int endIndex);

    /**
     * Sets selected LEDs to given color.
     * 
     * @param ledColor   Color to set.
     * @param startIndex Start of LED selection.
     * @param endIndex   End of LED selection.
     */
    public default void setLEDsInRange(Color ledColor, int startIndex, int endIndex) {
        Color8Bit col = new Color8Bit(ledColor);
        setLEDsInRange(col.red, col.green, col.blue, startIndex, endIndex);
    }

    /**
     * Sets selected LEDs to given color.
     * 
     * @param ledColor   Color to set.
     * @param startIndex Start of LED selection.
     * @param endIndex   End of LED selection.
     */
    public default void setLEDsInRange(Color8Bit ledColor, int startIndex, int endIndex) {
        setLEDsInRange(ledColor.red, ledColor.green, ledColor.blue, startIndex, endIndex);
    }

    /**
     * Set single LED to given color.
     * 
     * @param r     Red value.
     * @param g     Green value.
     * @param b     Blue value.
     * @param index LED index.
     */
    public abstract void setLED(int r, int g, int b, int index);

    /**
     * Set single LED to given color.
     * 
     * @param ledColor Color to set.
     * @param index    LED index.
     */
    public default void setLED(Color ledColor, int index) {
        Color8Bit col = new Color8Bit(ledColor);
        setLED(col.red, col.green, col.blue, index);
    }

    /**
     * Set single LED to given color.
     * 
     * @param ledColor Color to set.
     * @param index    LED index.
     */
    public default void setLED(Color8Bit ledColor, int index) {
        setLED(ledColor.red, ledColor.green, ledColor.blue, index);
    }

    /**
     * Sets lights to perform given animation.
     * 
     * @param state Animation to perform.
     */
    public abstract void setAnimation(BreakerAnimation state);

    /**
     * Sets lights to perform given animation.
     * 
     * @param startIndex LED index to start animation at.
     * @param animation      Animation to perform.
     */
    public abstract void setAnimation(int startIndex, BreakerAnimation animation);

    /**
     * Sets lights to perform given animation.
     * 
     * @param startIndex LED index to start animation at.
     * @param endIndex   LED index to end animation at.
     * @param animation      Animation to perform.
     */
    public abstract void setAnimation(int startIndex, int endIndex, BreakerAnimation animation);

    /**
     * Sets default LED state for given robot operating mode.
     * 
     * @param mode  Mode to assign state to.
     * @param state LED state to assign to given operating mode.
     */
    public abstract void setRobotModeDefaultState(RobotOperatingMode mode, BreakerAnimation state);

    /** clears all active effects to the set mode default */
    public abstract void clearToModeDefault();

    /** Clears current animation. */
    public abstract void clearActiveAnimation();

    /** Turns on LEDs. */
    public abstract void setOn();

    /** Turns off LEDs. */
    public abstract void setOff();

    /** @return Whether the lights are on or off. */
    public abstract boolean isOn();

    /** @return Whether robot lights are in default mode. */
    public abstract boolean isInModeDefault();

}
