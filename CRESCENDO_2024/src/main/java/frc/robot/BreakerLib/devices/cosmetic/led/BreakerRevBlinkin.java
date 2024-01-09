// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * Code interface for using the REV Blinkin LED controller. Different patterns
 * are assignable via methods with color/palette parameters.
 */
public class BreakerRevBlinkin {

    private Spark blinkin;

    /** Premade palettes for advanced fixed palette patterns. */
    public enum AdvancedPatternPalette {
        RAINBOW(0),
        PARTY(0.02),
        OCEAN(0.04),
        LAVA(0.06),
        FOREST(0.08);

        final double pwmVal;

        AdvancedPatternPalette(double pwmVal) {
            this.pwmVal = pwmVal;
        }
    }

    /** Fixed palette patterns utilizing a chosen {@link AdvancedPatternPalette}. */
    public enum AdvancedPattern {
        RAINBOW(-0.99),
        SINELON(-0.79),
        BEATS_PER_MIN(-0.69),
        TWINKLES(-0.55),
        COLOR_WAVES(-0.45);

        final double pwmVal;

        AdvancedPattern(double pwmVal) {
            this.pwmVal = pwmVal;
        }
    }

    /**
     * Fixed palette patterns which do not share color palettes with other patterns.
     */
    public enum FixedPalettePattern {
        RAINBOW_GLITTER(-0.89),
        CONFETTI(-0.87),
        SHOT_RED(-0.85),
        SHOT_BLUE(-0.83),
        SHOT_WHITE(-0.81),
        FIRE_MEDIUM(-0.59),
        FIRE_LARGE(-0.57),
        LARSON_RED(-0.35),
        LARSON_GRAY(-0.33),
        LIGHT_CHASE_RED(-0.31),
        LIGHT_CHASE_BLUE(-0.29),
        LIGHT_CHASE_GRAY(-0.27),
        HEARBEAT_RED(-0.25),
        HEARTBEAT_BLUE(-0.23),
        HEARTBEAT_WHITE(-0.21),
        HEARTBEAT_GRAY(-0.19),
        BREATH_RED(-0.17),
        BREATH_BLUE(-0.15),
        BREATH_GRAY(-0.13),
        STROBE_BLUE(-0.09),
        STROBE_GOLD(-0.07),
        STROBE_WHITE(-0.05);

        final double pwmVal;

        FixedPalettePattern(double pwmVal) {
            this.pwmVal = pwmVal;
        }
    }

    /**
     * Patterns which use one of the two colors selected on the device's
     * potentiometers.
     */
    public enum SingleColorPattern {
        END_BLEND_BLACK(-0.03),
        LARSON_SCANNER(-0.01),
        LIGHT_CHASE(0.01),
        HEARTBEAT_SLOW(0.03),
        HEARTBEAT_MED(0.05),
        HEARTBEAT_FAST(0.07),
        BREATH_SLOW(0.09),
        BREATH_FAST(0.11),
        SHOT(0.13),
        STROBE(0.15);

        final double pwmVal;

        SingleColorPattern(double pwmVal) {
            this.pwmVal = pwmVal;
        }
    }

    /** Select between Color 1 and Color 2. */
    public enum PotentiometerColor {
        COLOR_1(0),
        COLOR_2(0.2);

        final double pwmVal;

        PotentiometerColor(double pwmVal) {
            this.pwmVal = pwmVal;
        }
    }

    /** Patterns which use both Color 1 and Color 2. */
    public enum DualColorPattern {
        SPARKLE(0.37),
        SPARKLE_INV(0.39),
        GRADIENT(0.41),
        BEATS_PER_MIN(0.43),
        BLEND_1_TO_2(0.45),
        BLEND(0.47),
        NO_BLEND(0.49),
        TWINKLES(0.51),
        COLOR_WAVES(0.53),
        SINELON(0.55);

        final double pwmVal;

        DualColorPattern(double pwmVal) {
            this.pwmVal = pwmVal;
        }
    }

    /** Solid colors assignable to all LEDs at once. */
    public enum SolidColor {
        HOT_PINK(0.57),
        DARK_RED(0.59),
        RED(0.61),
        RED_ORANGE(0.63),
        ORANGE(0.65),
        GOLD(0.67),
        YELLOW(0.69),
        LAWN_GREEN(0.71),
        LIME(0.73),
        DARK_GREEN(0.75),
        GREEN(0.77),
        BLUE_GREEN(0.79),
        AQUA(0.81),
        SKY_BLUE(0.83),
        DARK_BLUE(0.85),
        BLUE(0.87),
        BLUE_VIOLET(0.89),
        VIOLET(0.91),
        WHITE(0.93),
        GRAY(0.95),
        DARK_GRAY(0.97),
        BLACK(0.99);

        final double pwmVal;

        SolidColor(double pwmVal) {
            this.pwmVal = pwmVal;
        }
    }

    /**
     * Constructs a REV Blinkin connected via PWM.
     * 
     * @param channel PWM channel to use.
     */
    public BreakerRevBlinkin(int channel) {
        blinkin = new Spark(channel);
    }

    /**
     * Set "advanced patterns" which all can use the same set of palettes.
     * 
     * @param palette Palette to use on pattern.
     * @param pattern Pattern to use.
     */
    public void setAdvancedPattern(AdvancedPatternPalette palette, AdvancedPattern pattern) {
        blinkin.set(palette.pwmVal + pattern.pwmVal);
    }

    /**
     * Set fixed palette patterns which do not fall under "advanced patterns".
     * 
     * @param pattern Pattern to use.
     */
    public void setFixedPalettePattern(FixedPalettePattern pattern) {
        blinkin.set(pattern.pwmVal);
    }

    /**
     * Sets a single-color pattern of choice.
     * 
     * @param color   Which potentiometer to receive the color from.
     * @param pattern Pattern to use.
     */
    public void setSingleColorPattern(PotentiometerColor color, SingleColorPattern pattern) {
        blinkin.set(pattern.pwmVal + color.pwmVal);
    }

    /**
     * Sets a dual colored pattern of choice.
     * 
     * @param pattern Pattern to use.
     */
    public void setDualColorPattern(DualColorPattern pattern) {
        blinkin.set(pattern.pwmVal);
    }

    /**
     * Sets solid color of choice.
     * 
     * @param color Color to use.
     */
    public void setSolidColor(SolidColor color) {
        blinkin.set(color.pwmVal);
    }

}
