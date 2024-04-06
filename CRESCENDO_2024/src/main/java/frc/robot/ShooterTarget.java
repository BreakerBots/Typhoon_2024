// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolablePair;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerInterpolatingTreeMap;
import frc.robot.subsystems.Drive;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.util.Precision;

/** Add your docs here. */
public class ShooterTarget {
    private Translation3d blueTargetPoint;
    private final Drive drivetrain;
    public static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    private Function<Double, Double> angleFunction;
    private Function<Double, Double> speedFunction;
    private SmartSpoolConfig smartSpoolConfig;
    public ShooterTarget(Drive drivetrain, Translation3d blueTargetPoint, SmartSpoolConfig smartSpoolConfig, List<Entry<Double, FireingTableValue>> fireingTable) {
        this.blueTargetPoint = blueTargetPoint;
        this.drivetrain = drivetrain;
        this.smartSpoolConfig = smartSpoolConfig;
        initCurves(fireingTable);
    }

    public ShooterTarget(Drive drivetrain, Translation3d blueTargetPoint, SmartSpoolConfig smartSpoolConfig, BreakerVector2 fixedFireingVec) {
        this.blueTargetPoint = blueTargetPoint;
        this.drivetrain = drivetrain;
        this.smartSpoolConfig = smartSpoolConfig;
        angleFunction = (Double distance) -> fixedFireingVec.getVectorRotation().getRadians();
        speedFunction = (Double distance) -> fixedFireingVec.getMagnitude();
    }

     private void initCurves(List<Entry<Double, FireingTableValue>> fireingTable) {
        double[] distances = new double[fireingTable.size()];
        double[] flywheelSpeeds = new double[fireingTable.size()];
        double[] angles = new double[fireingTable.size()];
        double constantSpeed = 0.0;
        boolean isFlySpeedConstant = true;

        for (int i = 0; i < fireingTable.size(); i++) {
            distances[i] = fireingTable.get(i).getKey();
            flywheelSpeeds[i] = fireingTable.get(i).getValue().fireingVector.getMagnitude();
            angles[i] = fireingTable.get(i).getValue().fireingVector.getVectorRotation().getRadians();
            
            if (i < 1) {
                constantSpeed = flywheelSpeeds[0];
            } else if (isFlySpeedConstant) {
                isFlySpeedConstant = MathUtil.isNear(constantSpeed, flywheelSpeeds[i], 1e-5);
            }
        }

        PolynomialSplineFunction angleCurve = SPLINE_INTERPOLATOR.interpolate(distances, flywheelSpeeds);
        angleFunction = (Double distance) -> angleCurve.value(distance);
        final double constantSpeedFinal = constantSpeed;
        if (isFlySpeedConstant) {
            speedFunction = (Double distance) -> constantSpeedFinal;
        } else {
            PolynomialSplineFunction speedCurve = SPLINE_INTERPOLATOR.interpolate(distances, angles);
            speedFunction = (Double distance) -> speedCurve.value(distance);
        }
    }

    

    public static record FireingTableValue(BreakerVector2 fireingVector, double time) {}
    public static record SmartSpoolConfig(double flywheelSpoolThreshold, double pitchTrackThreshold) {}



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

        double targetSpeed = speedFunction.apply(distance);
        double targetPivotAng = angleFunction.apply(distance);

        BreakerVector2 fireingVec = new BreakerVector2(Rotation2d.fromRadians(targetPivotAng), targetSpeed);
        BreakerLog.recordOutput("Distance To Target", distance);
        return new FireingSolution(deltaTransVecAng, distance, smartSpoolConfig, fireingVec);
    }

    public static record FireingSolution(Rotation2d yaw, double distanceToTarget, SmartSpoolConfig smartSpoolConfig, BreakerVector2 fireingVec){}


}
