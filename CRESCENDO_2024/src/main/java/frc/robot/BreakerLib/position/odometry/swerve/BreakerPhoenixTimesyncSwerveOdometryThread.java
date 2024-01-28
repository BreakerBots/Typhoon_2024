
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerPhoenixTimesyncSwerveOdometryThread extends BreakerSwerveOdometryThread {
    private CTREGyroYawStatusSignals gyroStatusSignals;
    private CTRESwerveModuleStatusSignals[] moduleStatusSignals;
    private BaseStatusSignal[] allStatusSignals;
    public BreakerPhoenixTimesyncSwerveOdometryThread(
            BreakerSwerveOdometryConfig odometryConfig, 
            CTREGyroYawStatusSignals gyroStatusSignals, 
            CTRESwerveModuleStatusSignals... moduleStatusSignals
        ) {
        super(odometryConfig);
        this.gyroStatusSignals = gyroStatusSignals;
        this.moduleStatusSignals = moduleStatusSignals;
        ArrayList<BaseStatusSignal> statSigArrList = new ArrayList<>();
        statSigArrList.add(gyroStatusSignals.gyroYaw);
        statSigArrList.add(gyroStatusSignals.gyroYawVel);
        for (CTRESwerveModuleStatusSignals swerveModSigs : moduleStatusSignals) {
            statSigArrList.add(swerveModSigs.drivePos);
            statSigArrList.add(swerveModSigs.driveVel);
            statSigArrList.add(swerveModSigs.turnPosAbs);
            statSigArrList.add(swerveModSigs.turnVel);
        }
        allStatusSignals = statSigArrList.toArray(new BaseStatusSignal[statSigArrList.size()]);
        
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
        SwerveModulePosition[] modPoses = getLatancyCompensatedModulePositions();
        double latancyCompYaw = MathUtil.inputModulus(BaseStatusSignal.getLatencyCompensatedValue(gyroStatusSignals.gyroYaw, gyroStatusSignals.gyroYawVel), -180.0, 180.0);
        Pose2d newPose = poseEstimator.update(Rotation2d.fromDegrees(latancyCompYaw), modPoses);
        robotRelSpeeds = drivetrain.getKinematics().toChassisSpeeds(drivetrain.getSwerveModuleStates());
        fieldRelSpeeds = BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), newPose.getRotation());
    }

    private SwerveModulePosition[] getLatancyCompensatedModulePositions() {
        SwerveModulePosition[] swerveModPoses = new SwerveModulePosition[moduleStatusSignals.length];
        for (int i = 0; i < moduleStatusSignals.length; i++) {
            CTRESwerveModuleStatusSignals swerveModSigs = moduleStatusSignals[i];
            double modAng = MathUtil.inputModulus(BaseStatusSignal.getLatencyCompensatedValue(swerveModSigs.turnPosAbs, swerveModSigs.turnVel), -0.5, 0.5);
            double modPos = BaseStatusSignal.getLatencyCompensatedValue(swerveModSigs.drivePos, swerveModSigs.driveVel) * swerveModSigs.wheelCircumfrenceMeters;
            swerveModPoses[i] = new SwerveModulePosition(modPos, Rotation2d.fromRotations(modAng));
        }
        return swerveModPoses;
    }

    public record CTRESwerveModuleStatusSignals(StatusSignal<Double> drivePos, StatusSignal<Double> driveVel, StatusSignal<Double> turnPosAbs, StatusSignal<Double> turnVel, double wheelCircumfrenceMeters) {}

    public record CTREGyroYawStatusSignals(StatusSignal<Double> gyroYaw, StatusSignal<Double> gyroYawVel) {}
}