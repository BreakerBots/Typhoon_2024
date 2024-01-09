// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public interface BreakerGenericParametricEquation {
    public Translation2d getValueAtT(double t);
}
