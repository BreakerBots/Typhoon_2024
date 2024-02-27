// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolablePair;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerInterpolatingTreeMap;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.interpolation.Interpolatable;

/** Add your docs here. */
public class ShooterTarget {
    private Translation3d blueTargetPoint;
    private final Drive drivetrain;
    private final BreakerInterpolatingTreeMap<Double, BreakerInterpolablePair<BreakerVector2, BreakerInterpolableDouble>> fireingTable;
    public ShooterTarget(Drive drivetrain, Translation3d blueTargetPoint, BreakerInterpolatingTreeMap<Double, BreakerInterpolablePair<BreakerVector2, BreakerInterpolableDouble>> fireingTable) {
        this.blueTargetPoint = blueTargetPoint;
        this.drivetrain = drivetrain;
        this.fireingTable = fireingTable;
    }

    public double getDistance2d() {
        return drivetrain.getOdometryPoseMeters().getTranslation().getDistance(getTargetPoint().toTranslation2d());
    }   

    public Translation3d getTargetPoint() {
        Optional<Alliance> allainceOpt =  DriverStation.getAlliance();
        if (allainceOpt.isPresent()) {
            if (allainceOpt.get() == Alliance.Red) {
                final double halfFieldWidth = 16.541/2.0;//Constants.FieldConstants.FIELD_WIDTH
                double deltaX = halfFieldWidth - -0.29209999; //blueTargetPoint.getX();
                double redX = halfFieldWidth + deltaX;
                return new Translation3d(redX, blueTargetPoint.getY(), blueTargetPoint.getZ());
            }
        }
        return blueTargetPoint; 
    }

    public FireingSolution getFireingSolution() {
        Translation2d drivetrainTrans = drivetrain.getOdometryPoseMeters().getTranslation();
        Translation2d targetTrans = getTargetPoint().toTranslation2d();
        Translation2d deltaTrans = drivetrainTrans.minus(targetTrans);
        Rotation2d deltaTransVecAng = deltaTrans.getAngle();
        double distance = drivetrainTrans.getDistance(targetTrans);
        BreakerVector2 fireingVec = fireingTable.getInterpolatedValue(distance).getFirst();
        BreakerLog.recordOutput("Distance To Target", distance);
        return new FireingSolution(deltaTransVecAng, fireingVec);
    }

    public static record FireingSolution(Rotation2d yaw, BreakerVector2 fireingVec){}


}
