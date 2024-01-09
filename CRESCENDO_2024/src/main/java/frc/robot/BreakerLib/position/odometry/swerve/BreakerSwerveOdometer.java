// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.BreakerOdometryLoop;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.math.BreakerMath;

public class BreakerSwerveOdometer implements BreakerGenericOdometer {
    private SwerveDriveOdometry odometry;
    private BreakerSwerveDrive drivetrain;
    private ReentrantLock odometryLock = new ReentrantLock();
    private BreakerOdometryLoop odometryLoop;
    private BreakerMovementState2d prevMovementState = new BreakerMovementState2d(),
      curMovementState = new BreakerMovementState2d();
    private ChassisSpeeds robotRelSpeeds = new ChassisSpeeds(), fieldRelSpeeds = new ChassisSpeeds();
    private double prevOdometryUpdateTimestamp = 0;
    public BreakerSwerveOdometer(BreakerSwerveDrive drivetrain, Pose2d initialPose) {
        this.drivetrain = drivetrain;
        odometry = new SwerveDriveOdometry(drivetrain.getKinematics(), drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions(), initialPose);
        odometryLoop = new BreakerOdometryLoop(this::updateOdometry);
    }

    public BreakerSwerveOdometer(BreakerSwerveDrive drivetrain) {
       this(drivetrain, new Pose2d());
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
      odometryLock.lock();
      odometry.resetPosition(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions(), newPose);
      odometryLock.unlock();
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
      odometryLock.lock(); 
      Pose2d pos = odometry.getPoseMeters();
      odometryLock.unlock();
      return pos;
    }

    @Override
    public BreakerMovementState2d getMovementState() {
      return curMovementState;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
      return robotRelSpeeds;
    }
  
    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
      return fieldRelSpeeds;
    }

    private void calculateMovementState(double timeToLastUpdateMiliseconds) {
        curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(),
            getFieldRelativeChassisSpeeds(), timeToLastUpdateMiliseconds, prevMovementState);
        prevMovementState = curMovementState;
    }

    private void updateOdometry() {
      odometryLock.lock();
        odometry.update(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions());
        calculateMovementState((Timer.getFPGATimestamp() - prevOdometryUpdateTimestamp) * 1000);
        prevOdometryUpdateTimestamp = Timer.getFPGATimestamp();
        robotRelSpeeds = drivetrain.getKinematics().toChassisSpeeds(drivetrain.getSwerveModuleStates());
        fieldRelSpeeds = BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), odometry.getPoseMeters().getRotation());
      odometryLock.unlock();
    }

    @Override
    public void toLog(LogTable table) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'toLog'");
    }

    @Override
    public BreakerOdometryLoop getOdometryLoop() {
      return odometryLoop;
    }
    
  }
