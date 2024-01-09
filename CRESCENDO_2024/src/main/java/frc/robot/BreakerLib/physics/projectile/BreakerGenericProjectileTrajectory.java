// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.projectile;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;

/** Add your docs here. */
public interface BreakerGenericProjectileTrajectory {
    public abstract BreakerVector3 getInitialVels();
    public abstract Translation3d getLaunchPosition();
    public abstract Translation3d getPosition(double t);
    public abstract Translation3d getDisplacement(double t);
    public abstract Translation2d getPosition2d(double t);
    public abstract Translation2d getDisplacement2d(double t);
    public abstract BreakerVector3 getVelocity(double t);
    public abstract double getTimeToGivenDisplacemntMagnitude2d(double displacementMag);
    public abstract Translation2d getMovingLaunchCorrectionAsNewTargetLocation(ChassisSpeeds fieldRelativeSpeeds,
            Translation2d targetLocation);
    public abstract BreakerVector3 getMovingLaunchCorrectionAsNewLaunchForces(ChassisSpeeds fieldRelativeSpeeds,
            Translation2d targetLocation);
}
