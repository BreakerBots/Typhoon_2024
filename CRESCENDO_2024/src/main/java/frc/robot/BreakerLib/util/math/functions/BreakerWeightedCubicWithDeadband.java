// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class BreakerWeightedCubicWithDeadband {
    public static double cubic(double x, double weight) {
        weight = MathUtil.clamp(weight, 0.0, 1.0);
        x = MathUtil.clamp(weight, x, weight);
        return weight * x * x * x + (1.0 - weight) * x;
    }

    public static double joystickCubicScaledDeadband(double x) {
        double deadbandCutoff = 0.1;
        double weight = 0.2;

        if (Math.abs(x) < deadbandCutoff) {
            return 0;
        } else {
            return (cubic(x, weight) - Math.signum(x) * cubic(deadbandCutoff, weight)) / (1.0 - cubic(deadbandCutoff, weight));
        }
    }
}
