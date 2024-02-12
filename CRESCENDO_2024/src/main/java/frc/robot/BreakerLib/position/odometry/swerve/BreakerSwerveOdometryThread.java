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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerOdometryThread;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPoseSource;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseCordinateSystem;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.math.BreakerMath;

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
    protected ChassisSpeeds robotRelSpeeds, fieldRelSpeeds;
    protected BreakerSwerveOdometryConfig odometryConfig;
    public BreakerSwerveOdometryThread(BreakerSwerveOdometryConfig odometryConfig) {
        super(odometryConfig.threadPriority);
        robotRelSpeeds = new ChassisSpeeds();
        fieldRelSpeeds = new ChassisSpeeds();
        defaultVisionDevs = odometryConfig.defaultVisionStdDevs;
        odometeryPeriod = odometryConfig.odometeryPeriod;
        poseOrigin = odometryConfig.poseOrigin;
        this.odometryConfig = odometryConfig;
        estimatedPoseSources = new ArrayList<>();
        
    }

    @Override
    public synchronized void start() {}

    //TODO make this a better solution
    public synchronized void start(BreakerSwerveDrive drivetrain) {
        this.drivetrain = drivetrain;
        poseEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(), drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions(), odometryConfig.initialPoseMeters, odometryConfig.stateStdDevs, odometryConfig.defaultVisionStdDevs);
        super.start();
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
        boolean safeBool = false;
        odometryLock.lock();
        safeBool = visionSeeded;
        odometryLock.unlock();
        return safeBool;
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
    Pose2d safePose = new Pose2d();
    odometryLock.lock();
    safePose = poseEstimator.getEstimatedPosition();
    odometryLock.unlock();
    return safePose;
  }

  /** @return Movement state of object. */
  public BreakerMovementState2d getMovementState() {
   // TODO
    return new BreakerMovementState2d();
  }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        odometryLock.lock();
        poseEstimator.resetPosition(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions(), newPose);
        odometryLock.unlock();
    }

    @Override
    public void toLog(LogTable table) {
        //table.put("Pose", Pose2d.struct, getOdometryPoseMeters());
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
        checkRegisteredPoseSources();
        Pose2d newPose = poseEstimator.update(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions());
        robotRelSpeeds = drivetrain.getKinematics().toChassisSpeeds(drivetrain.getSwerveModuleStates());
        fieldRelSpeeds = BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), newPose.getRotation());
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return robotRelSpeeds;
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return fieldRelSpeeds;
    }

    /**
     * InnerBreakerSwerveOdometryThread
     */
    public record BreakerSwerveOdometryConfig(double odometeryPeriod, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> defaultVisionStdDevs, PoseOrigin poseOrigin, Pose2d initialPoseMeters, int threadPriority) {
        public BreakerSwerveOdometryConfig() {
            this(1.0/200.0,  VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9), PoseOrigin.ofGlobal(), new Pose2d(), 1);
        }
    }
}
