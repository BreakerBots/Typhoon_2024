// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.projectile.BreakerProjectileTrajectory;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerInterpolatingTreeMap;

/** Robot turret system. */
public class BreakerTurretController {

    private BreakerInterpolatingTreeMap<Double,BreakerVector2> firingTable;
    private BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> rpmToProjectileLaunchVelocity;

    /** @param projectile
     *  @param firingTable Firing distance compared to BreakerVector2(X: angle rad, Y: RPM)
     *  @param rpmToProjectileLaunchVelocity Interpolating table that relates flywheel RPM to the launch velocity of the projectile.
      */
    public BreakerTurretController(BreakerInterpolatingTreeMap<Double,BreakerVector2> firingTable, BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> rpmToProjectileLaunchVelocity) {
        this.firingTable = firingTable;
        this.rpmToProjectileLaunchVelocity = rpmToProjectileLaunchVelocity;
    }

    
    /** 
     * @param projectileLaunchPointRelativeToField
     * @param targetPointRelativeToField
     * @return BreakerTurretState
     */
    public BreakerTurretState calculateFieldRelative(Translation3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField) {
        double distance = projectileLaunchPointRelativeToField.toTranslation2d().getDistance(targetPointRelativeToField.toTranslation2d());
        BreakerVector2 firingSolution = firingTable.getInterpolatedValue(distance);

        Rotation2d azAng = BreakerMath.getPointAngleRelativeToOtherPoint(projectileLaunchPointRelativeToField.toTranslation2d(), targetPointRelativeToField.toTranslation2d());
        Rotation2d pitAng = firingSolution.getVectorRotation();

       return new BreakerTurretState(azAng, pitAng, firingSolution.getMagnitude());
    }

    public BreakerTurretState calculateCompensatedFieldRelative(Translation3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField, ChassisSpeeds fieldRelativeChassisSpeeds) {
        double distance = projectileLaunchPointRelativeToField.toTranslation2d().getDistance(targetPointRelativeToField.toTranslation2d());
        BreakerVector2 firingSolution = firingTable.getInterpolatedValue(distance);

        Rotation2d azAng = BreakerMath.getPointAngleRelativeToOtherPoint(projectileLaunchPointRelativeToField.toTranslation2d(), targetPointRelativeToField.toTranslation2d());
        Rotation2d pitAng = new Rotation2d(firingSolution.getX());
        
        double launchVel = rpmToProjectileLaunchVelocity.getInterpolatedValue(firingSolution.getY()).getValue();
        BreakerVector3 launchVec = new BreakerVector3(launchVel, new Rotation3d(0.0, pitAng.getRadians(), azAng.getRadians()));
        BreakerProjectileTrajectory predictedTrajectory = new BreakerProjectileTrajectory(launchVec, projectileLaunchPointRelativeToField);
        Translation2d correctedTargetTrans2d = predictedTrajectory.getMovingLaunchCorrectionAsNewTargetLocation(fieldRelativeChassisSpeeds, targetPointRelativeToField.toTranslation2d());
        Translation3d correctedTargetTrans3d = new Translation3d(correctedTargetTrans2d.getX(), correctedTargetTrans2d.getY(), targetPointRelativeToField.getZ());
    
       return calculateFieldRelative(projectileLaunchPointRelativeToField, correctedTargetTrans3d);
    }

    
    /** 
     * @param projectileLaunchPointRelativeToField
     * @param targetPointRelativeToField
     * @return BreakerTurretState
     */
    public BreakerTurretState calculateRobotRelative(Pose3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField) {
        BreakerTurretState calcState = calculateFieldRelative(projectileLaunchPointRelativeToField.getTranslation(), targetPointRelativeToField);
        return calcState.toRobotRelativeState(projectileLaunchPointRelativeToField.getRotation());
    }

    public BreakerTurretState calculateCompensatedRobotRelative(Pose3d projectileLaunchPointRelativeToField, Translation3d targetPointRelativeToField, ChassisSpeeds fieldRelativeChassisSpeeds) {
        BreakerTurretState calcState = calculateCompensatedFieldRelative(projectileLaunchPointRelativeToField.getTranslation(), targetPointRelativeToField, fieldRelativeChassisSpeeds);
        return calcState.toRobotRelativeState(projectileLaunchPointRelativeToField.getRotation());
    }
}