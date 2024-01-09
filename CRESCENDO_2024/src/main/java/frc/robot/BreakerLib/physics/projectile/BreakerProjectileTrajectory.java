// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.projectile;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/**
 * Represents a drag affected object's balisitc trajectory without external
 * propulsion (note: force vector matches field cordnate system and is field
 * relative (EX: Z is up))
 */
public class BreakerProjectileTrajectory implements BreakerGenericProjectileTrajectory {
    private BreakerVector3 initialVels;
    private Translation3d  launchPoint;

    public BreakerProjectileTrajectory(BreakerVector3 initialVels, Translation3d launchPoint) {
        this.initialVels = initialVels;
        this.launchPoint = launchPoint;
    }

    public BreakerProjectileTrajectory(double initialVel, Pose3d launchPose) {
        this.initialVels = new BreakerVector3(initialVel, launchPose.getRotation());
        launchPoint = launchPose.getTranslation();
    }
    

    private double getLinearDislacementAtGivenTime(double initialVel, double time) {
        return initialVel * time;
    }

    private double getVeticalDisplacementAtGivenTime(double initialVel, double time) {
        return (initialVel * time) - ((0.5 * BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G) * (time * time));
    }

    public Translation3d getDisplacement(double t) {
        return new Translation3d(getLinearDislacementAtGivenTime(initialVels.getX(), t), getLinearDislacementAtGivenTime(initialVels.getY(), t), getVeticalDisplacementAtGivenTime(initialVels.getZ(),t ));
    }

    public Translation3d getPosition(double t) {
        return launchPoint.plus(getDisplacement(t));
    }

    public Translation2d getDisplacement2d(double t) {
        return new Translation2d(getLinearDislacementAtGivenTime(initialVels.getX(), t), getLinearDislacementAtGivenTime(initialVels.getY(), t));
    }

    public Translation2d getPosition2d(double t) {
        return getPosition(t).toTranslation2d();
    }

    public BreakerVector3 getVelocity(double t) {
        return new BreakerVector3(initialVels.getX(), initialVels.getY(), initialVels.getZ() - ((0.5 * BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G) * (t * t)));
    }

    public double getTimeToGivenDisplacemntMagnitude2d(double displacementMag) {
        return displacementMag / initialVels.toBreakerVector2().getMagnitude();
    }

    /** Creates a corrected 2D pose for the target (I.E. where you should aim) based on chassis movement. */
    public Translation2d getMovingLaunchCorrectionAsNewTargetLocation(ChassisSpeeds fieldRelativeSpeeds,
            Translation2d targetLocation) {
        BreakerProjectileTrajectory trajectory = new BreakerProjectileTrajectory(
                new BreakerVector3(initialVels.getX() + fieldRelativeSpeeds.vxMetersPerSecond,
                        initialVels.getY() + fieldRelativeSpeeds.vyMetersPerSecond, initialVels.getZ()),
                launchPoint);
        double displacementToTarget = launchPoint.toTranslation2d().getDistance(targetLocation);
        double timeToTarget = getTimeToGivenDisplacemntMagnitude2d(displacementToTarget);
        Translation2d predictedImpactPos = trajectory.getDisplacement2d(timeToTarget);
        Translation2d tgtDiff = predictedImpactPos.minus(targetLocation);
        return targetLocation.minus(tgtDiff);
    }

    /** Creates a corrected 3D vector for the projectile (I.E. corrected launch forces) based on movement of chassis. */
    public BreakerVector3 getMovingLaunchCorrectionAsNewLaunchForces(ChassisSpeeds fieldRelativeSpeeds,
            Translation2d targetLocation) {
        BreakerProjectileTrajectory trajectory = new BreakerProjectileTrajectory(
                new BreakerVector3(initialVels.getX() + fieldRelativeSpeeds.vxMetersPerSecond,
                        initialVels.getY() + fieldRelativeSpeeds.vyMetersPerSecond, initialVels.getZ()),
                launchPoint);
        double displacementToTarget = launchPoint.toTranslation2d().getDistance(targetLocation);
        double baseExpectedImpactTime = getTimeToGivenDisplacemntMagnitude2d(displacementToTarget);
        BreakerVector3 baseExpectedImpactVec = getVelocity(baseExpectedImpactTime);
        BreakerVector3 predictedImpactVec = trajectory.getVelocity(baseExpectedImpactTime);
        BreakerVector3 correctionVec = baseExpectedImpactVec.minus(predictedImpactVec);
        return initialVels.plus(correctionVec);
    }

    @Override
    public BreakerVector3 getInitialVels() {
        return initialVels;
    }

    @Override
    public Translation3d getLaunchPosition() {
        return launchPoint;
    }
}
