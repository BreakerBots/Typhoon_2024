// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.BreakerLib.control.statespace.BreakerFlywheelStateSpace;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerOdometryThread;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPoseSource;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseCordinateSystem;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;

/** Add your docs here. */
public class BreakerSwerveOdometryThread extends BreakerOdometryThread {
    protected SwerveDrivePoseEstimator poseEstimator;
    protected ArrayList<BreakerEstimatedPoseSource> estimatedPoseSources;
    protected Matrix<N3, N1> defaultVisionDevs;
    protected static final Matrix<N3, N1> DOUBLE_MAX_VAL_MATRIX = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    protected PoseOrigin poseOrigin;
    protected boolean visionSeeded;
    protected BreakerSwerveDrive drivetrain;
    protected double odometeryPeriod;
    public BreakerSwerveOdometryThread(BreakerSwerveDrive drivetrain, double odometeryPeriod, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> defaultVisionStdDevs, PoseOrigin poseOrigin, Pose2d initialPoseMeters, int threadPriority) {
        super(threadPriority);
        poseEstimator = new SwerveDrivePoseEstimator(null, null, null, null, null, null);
    }

    public Matrix<N3, N1> getDefaultVisionStdDevs() {
        return defaultVisionDevs;
    }

    public void registerEstimatedPoseSource(BreakerEstimatedPoseSource estimatedPoseSourceToAdd) {
        odometryLock.lock();
        estimatedPoseSources.add(estimatedPoseSourceToAdd);
        odometryLock.unlock();
    }

    public void registerEstimatedPoseSources(BreakerEstimatedPoseSource... estimatedPoseSourcesToAdd) {
        odometryLock.lock();
        for (BreakerEstimatedPoseSource eps : estimatedPoseSourcesToAdd) {
            estimatedPoseSources.add(eps);
        }
        odometryLock.unlock();
    }

    public void addVisionPoseEstimate(BreakerEstimatedPose estimatedPose) {
        odometryLock.lock();
        Optional<PoseCordinateSystem> cordSysOpt = poseOrigin.getCordinateSystem();
        if (cordSysOpt.isPresent()) {
            PoseCordinateSystem chordSys = cordSysOpt.get();
            privateAddVisionPoseEstimate(new BreakerEstimatedPose(estimatedPose, chordSys));
        }
        odometryLock.unlock();
    }

    protected void privateAddVisionPoseEstimate(BreakerEstimatedPose estimatedPose) {
        Optional<Matrix<N3, N1>> estStdDevsOpt = estimatedPose.estimationStandardDevations;
        Matrix<N3, N1> stdDevs =  defaultVisionDevs;
        if (estStdDevsOpt.isPresent()) {
            Matrix<N3, N1> estStdDevs = estStdDevsOpt.get();
            if (estStdDevs.isEqual(DOUBLE_MAX_VAL_MATRIX, 1e-5)) {
                return;
            }
            stdDevs = estStdDevs;
        }
        if (!visionSeeded) {
            poseEstimator.resetPosition(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions(), getOdometryPoseMeters());
        } else {
            poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.captureTimestamp, stdDevs);
        }
       
    }

    public boolean hasBeenVisionSeeded() {
        return visionSeeded;
    }

    protected void checkRegisteredPoseSources() {
        for (BreakerEstimatedPoseSource estPoseSource : estimatedPoseSources) {
            Optional<BreakerEstimatedPose> estPoseOpt = estPoseSource.getEstimatedPose(poseOrigin);
            if (estPoseOpt.isPresent()) {
                privateAddVisionPoseEstimate(estPoseOpt.get());
            }
        }
    }

    /** @return Odometery pose in meters. */
  public Pose2d getOdometryPoseMeters() {

  }

  /** @return Movement state of object. */
  public BreakerMovementState2d getMovementState() {

  }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        
    }

    @Override
    public void toLog(LogTable table) {

    }

    @Override
    protected void init() {
        drivetrain.setStatusUpdatePeriod(odometeryPeriod);
    }

    @Override
    protected void periodControl() {
        try {
            Thread.sleep((long)(odometeryPeriod * 1000));
        } catch (InterruptedException ex) {}
    
    }

    @Override
    protected void update() {
        poseEstimator.update(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions());
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRobotRelativeChassisSpeeds'");
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFieldRelativeChassisSpeeds'");
    }
}
