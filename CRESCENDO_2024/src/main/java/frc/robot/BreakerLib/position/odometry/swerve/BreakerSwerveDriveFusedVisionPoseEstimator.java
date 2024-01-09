// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSource;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSource.BreakerEstimatedPose;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;

/** Estimates swerve drive pose based on vision odometry. */
public class BreakerSwerveDriveFusedVisionPoseEstimator
        extends SubsystemBase implements BreakerGenericOdometer {
    private BreakerSwerveDrivePoseEstimator poseEstimator;
    private BreakerEstimatedPoseSource[] poseSources;


    public BreakerSwerveDriveFusedVisionPoseEstimator(
            BreakerSwerveDrive drivetrain,
            Pose2d initialPose,
            Matrix<N3, N1> stateStandardDeviation,
            Matrix<N3, N1> defaultVisionStanderdDeveation,
            BreakerEstimatedPoseSource... poseSources) {
        
        this.poseSources = poseSources;
        poseEstimator = new BreakerSwerveDrivePoseEstimator(
                drivetrain, initialPose,
                stateStandardDeviation, defaultVisionStanderdDeveation);
    }

    public void changeDefaultVisionDevs(Matrix<N3, N1> defaultVisionStanderdDeveation) {
       poseEstimator.changeDefaultVisionDevs(defaultVisionStanderdDeveation);
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        poseEstimator.setOdometryPosition(newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return poseEstimator.getOdometryPoseMeters();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return poseEstimator.getMovementState();
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return poseEstimator.getRobotRelativeChassisSpeeds();
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return poseEstimator.getFieldRelativeChassisSpeeds();
    }

    private void updateOdometry() {
        for (BreakerEstimatedPoseSource poseSource: poseSources) {
            Optional<BreakerEstimatedPose> estPose = poseSource.getEstimatedPose(null);
            if (estPose.isPresent()) {
                poseEstimator.addVisionMeasurment(estPose.get());
            }
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    @Override
    public void toLog(LogTable table) {
        BreakerGenericOdometer.super.toLog(table);
        poseEstimator.toLog(table.getSubtable("BasePoseEstimator"));
    }
}
