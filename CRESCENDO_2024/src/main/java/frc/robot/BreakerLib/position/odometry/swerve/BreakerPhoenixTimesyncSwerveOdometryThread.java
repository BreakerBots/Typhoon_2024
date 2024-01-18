
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerPhoenixTimesyncSwerveOdometryThread extends BreakerSwerveOdometryThread {
    private CTREGyroYawStatusSignals gyroStatusSignals;
    private CTRESwerveModuleStatusSignals[] moduleStatusSignals;
    private StatusSignal<Double>[] allStatusSignals;
    public BreakerPhoenixTimesyncSwerveOdometryThread(BreakerSwerveDrive drivetrain,
            BreakerSwerveOdometryConfig odometryConfig, CTREGyroYawStatusSignals gyroStatusSignals, CTRESwerveModuleStatusSignals... moduleStatusSignals) {
        super(drivetrain, odometryConfig);
        this.gyroStatusSignals = gyroStatusSignals;
        this.moduleStatusSignals = moduleStatusSignals;
        ArrayList<StatusSignal<Double>> statSigArrList = new ArrayList<>();
        statSigArrList.add(gyroStatusSignals.gyroYaw);
        statSigArrList.add(gyroStatusSignals.gyroYawVel);
        for (CTRESwerveModuleStatusSignals swerveModSigs : moduleStatusSignals) {
            statSigArrList.add(swerveModSigs.drivePos);
            statSigArrList.add(swerveModSigs.driveVel);
            statSigArrList.add(swerveModSigs.turnPosAbs);
            statSigArrList.add(swerveModSigs.turnVel);
        }
        Arrays.copy
        
    }

    @Override
    protected void periodControl() {
        odometryLock.lock();
        try {
            BaseStatusSignal.waitForAll(2.0 * odometeryPeriod, allStatusSignals);
        } catch (Exception e) {
           BreakerLog.logError(e);
        } finally {
            odometryLock.unlock();
        }
    }

    @Override
    protected void update() {
        checkRegisteredPoseSources();
        Pose2d newPose = poseEstimator.update(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getSwerveModulePositions());
        robotRelSpeeds = drivetrain.getKinematics().toChassisSpeeds(drivetrain.getSwerveModuleStates());
        fieldRelSpeeds = BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), newPose.getRotation());
    }

    

    public record CTRESwerveModuleStatusSignals(StatusSignal<Double> drivePos, StatusSignal<Double> driveVel, StatusSignal<Double> turnPosAbs, StatusSignal<Double> turnVel) {}

    public record CTREGyroYawStatusSignals(StatusSignal<Double> gyroYaw, StatusSignal<Double> gyroYawVel) {}
}