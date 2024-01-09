// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.projectile;

import java.util.ArrayList;
import java.util.Set;
import java.util.Map.Entry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerGenericInterpolatingMap;

/** Add your docs here. */
public class BreakerProjectileTrajectoryWithDrag implements BreakerGenericProjectileTrajectory {
    private BreakerVector3 initialVels;
    private Translation3d  launchPoint;
    private BreakerProjectile projectile;
    private double simulationTimestep, simulationEndHightZ;
    private BreakerGenericInterpolatingMap<Double, BreakerProjectileTrajectoryState> interpolationMap;

    public BreakerProjectileTrajectoryWithDrag(BreakerProjectile projectile, BreakerVector3 initialVels, Translation3d launchPoint, double simulationTimestep, double simulationEndHightZ, BreakerGenericInterpolatingMap<Double, BreakerProjectileTrajectoryState> interpolationMap) {
        this.initialVels = initialVels;
        this.launchPoint = launchPoint;
        this.projectile = projectile;
        
    }

    public BreakerProjectileTrajectoryWithDrag(BreakerProjectile projectile, double initialVel, Pose3d launchPose) {
        this(projectile, new BreakerVector3(initialVel, launchPose.getRotation()), launchPose.getTranslation());
    }

    protected void simulate() {
        double time = 0;
        BreakerProjectileTrajectoryState lastState = new BreakerProjectileTrajectoryState(initialVels);
        interpolationMap.put(0.0, lastState);
        while((lastState.getDisplacement().getZ() + launchPoint.getZ()) > simulationEndHightZ) {
            interpolationMap.put(time, lastState);
            BreakerProjectileTrajectoryState newState = lastState.calculateNext(projectile, simulationTimestep);
            time += simulationTimestep;
            lastState = newState;
        }
    }

    public static class BreakerProjectileTrajectoryState implements BreakerInterpolable<BreakerProjectileTrajectoryState> {
        private Translation3d displacement;
        private BreakerVector3 velocity;
        public BreakerProjectileTrajectoryState(Translation3d displacement, BreakerVector3 velocity) {

        }

        public BreakerProjectileTrajectoryState(BreakerVector3 initialVelocity) {

        }

        public Translation3d getDisplacement() {
            return displacement;
        }

        public BreakerVector3 getVelocity() {
            return velocity;
        }

        public BreakerProjectileTrajectoryState calculateNext(BreakerProjectile projectile, double simulationTimestep) {
            double dragDeltaVelX = projectile.getDragAccel(velocity.getX()) * simulationTimestep;
            double dragDeltaVelY = projectile.getDragAccel(velocity.getY()) * simulationTimestep;
            double dragDeltaVelZ = projectile.getDragAccel(velocity.getY()) * simulationTimestep;
            double totalDeltaVelZ = dragDeltaVelZ + (BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G*simulationTimestep);
            double finalVelX = velocity.getX() - dragDeltaVelX;
            double finalVelY = velocity.getY() - dragDeltaVelY;
            double finalVelZ = velocity.getZ() - totalDeltaVelZ;
            BreakerVector3 newVel = new BreakerVector3(finalVelX, finalVelY, finalVelZ);
            BreakerVector3 avgVel = newVel.plus(velocity).div(2.0);
            Translation3d displacement = avgVel.times(simulationTimestep).getAsTranslation();
            return new BreakerProjectileTrajectoryState(displacement, newVel);
        }

        @Override
        public BreakerProjectileTrajectoryState interpolate(BreakerProjectileTrajectoryState endValue, double t) {
            return new BreakerProjectileTrajectoryState(getDisplacement().interpolate(endValue.getDisplacement(), t), getVelocity().interpolate(endValue.getVelocity(), t));
        }

        @Override
        public double[] getInterpolatableData() {
            return new double[]{displacement.getX(), displacement.getY(), displacement.getZ(), velocity.getX(), velocity.getY(), velocity.getZ()};
        }

        @Override
        public BreakerProjectileTrajectoryState fromInterpolatableData(double[] interpolatableData) {
           return new BreakerProjectileTrajectoryState(new Translation3d(interpolatableData[0], interpolatableData[1], interpolatableData[2]), new BreakerVector3(interpolatableData[3], interpolatableData[4], interpolatableData[5]));
        }
    }

    @Override
    public BreakerVector3 getInitialVels() {
       return initialVels;
    }

    @Override
    public Translation3d getLaunchPosition() {
       return launchPoint;
    }

    @Override
    public Translation3d getPosition(double t) {
        return launchPoint.plus(getDisplacement(t));
    }

    @Override
    public Translation3d getDisplacement(double t) {
        return interpolationMap.getInterpolatedValue(t).getDisplacement();
    }

    @Override
    public Translation2d getPosition2d(double t) {
       return getPosition(t).toTranslation2d();
    }

    @Override
    public Translation2d getDisplacement2d(double t) {
       return getDisplacement(t).toTranslation2d();
    }

    @Override
    public BreakerVector3 getVelocity(double t) {
        return interpolationMap.getInterpolatedValue(t).getVelocity();
    }

    @Override
    public double getTimeToGivenDisplacemntMagnitude2d(double displacementMag) {
        Set<Entry<Double, BreakerProjectileTrajectoryState>> entSet = interpolationMap.entrySet();
        ArrayList<Entry<Double, BreakerProjectileTrajectoryState>> statesLessThan;

        throw new UnsupportedOperationException("Unimplemented method 'getTimeToGivenDisplacemntMagnitude2d'");
    }

    /** Creates a corrected 2D pose for the target (I.E. where you should aim) based on chassis movement. */
    public Translation2d getMovingLaunchCorrectionAsNewTargetLocation(ChassisSpeeds fieldRelativeSpeeds,
            Translation2d targetLocation) {
        BreakerProjectileTrajectoryWithDrag trajectory = new BreakerProjectileTrajectory(
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
        BreakerProjectileTrajectoryWithDrag trajectory = new BreakerProjectileTrajectory(
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
}
