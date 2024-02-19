// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class BreakerLinearizedConstrainedExponential extends BreakerMathFunction {
    /**https://www.desmos.com/calculator/epmml4xp0h */
    public BreakerLinearizedConstrainedExponential(double linearity, double baseExponent) {
        super((Double x) -> {return curve(x, linearity, baseExponent);});
    }

    /**https://www.desmos.com/calculator/epmml4xp0h */
    public BreakerLinearizedConstrainedExponential() {
        this(0.3, 3.0);
    }

    private static double curve(double x, double lineariy, double exp) {
        lineariy = MathUtil.clamp(lineariy, 0.0, 1.0);
        exp = Math.max(exp, 1.0);
        return ((1.0 - lineariy) * Math.pow(x, exp)) + (lineariy * x);
    }
}
