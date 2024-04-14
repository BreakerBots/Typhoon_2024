// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import edu.wpi.first.units.Measure;

/** Add your docs here. */
public class MechanismRatio {
    private final double ratioToOne;

    public MechanismRatio(double from, double to) {
        ratioToOne = to / from;
    }

    public MechanismRatio(double ratioToOne) {
        this.ratioToOne = ratioToOne;
    }

    public MechanismRatio add(MechanismRatio ratio) {
        return new MechanismRatio(ratioToOne * ratio.getRatioToOne());
    }

    public double getOutput(double input) {
        return input / ratioToOne;
    }

    public double getInput(double output) {
        return ratioToOne / output;
    }

    public double getRatioToOne() {
        return ratioToOne;
    }

    public double getInverseRatio() {
        return 1.0 / ratioToOne;
    }
}
