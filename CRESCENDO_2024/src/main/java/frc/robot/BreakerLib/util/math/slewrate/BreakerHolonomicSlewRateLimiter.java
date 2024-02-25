// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.slewrate;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest.ChassisPercentSpeeds;
import frc.robot.BreakerLib.util.math.slewrate.BreakerHolonomicSlewRateLimiter.UnitlessChassisSpeeds;

/** Add your docs here. */
public class BreakerHolonomicSlewRateLimiter {
    private double positiveLinearRateLimit, negitiveLinearRateLimit, positiveAngularRateLimit, negitiveAngularRateLimit;
    private UnitlessChassisSpeeds prevSpeeds;
    //private SlewRateLimiter omegaLimit, xLimit, yLimit;
    private double prevTime;
    public BreakerHolonomicSlewRateLimiter(double positiveLinearRateLimit, double negitiveLinearRateLimit, double positiveAngularRateLimit, double negitiveAngularRateLimit, UnitlessChassisSpeeds initalSpeeds) {
        prevTime = MathSharedStore.getTimestamp();
        prevSpeeds = initalSpeeds;
        // omegaLimit = new SlewRateLimiter(positiveAngularRateLimit, negitiveAngularRateLimit, initalSpeeds.omega);
        // xLimit = new SlewRateLimiter(positiveLinearRateLimit, negitiveLinearRateLimit, initalSpeeds.x);
        // yLimit = new SlewRateLimiter(positiveLinearRateLimit, negitiveLinearRateLimit, initalSpeeds.y);
        this.positiveLinearRateLimit = positiveLinearRateLimit;
        this.negitiveLinearRateLimit = negitiveLinearRateLimit;
        this.positiveAngularRateLimit = positiveAngularRateLimit;
        this.negitiveAngularRateLimit = negitiveAngularRateLimit;

    }

    public double getPositiveLinearRateLimit() {
        return positiveLinearRateLimit;
    }

    public double getNegitiveLinearRateLimit() {
        return negitiveLinearRateLimit;
    }

    public double getNegitiveAngularRateLimit() {
        return negitiveAngularRateLimit;
    }

    public double getPositiveAngularRateLimit() {
        return positiveAngularRateLimit;
    }

    public void setNegitiveAngularRateLimit(double negitiveAngularRateLimit) {
        this.negitiveAngularRateLimit = negitiveAngularRateLimit;
    }

    public void setNegitiveLinearRateLimit(double negitiveLinearRateLimit) {
        this.negitiveLinearRateLimit = negitiveLinearRateLimit;
    }

    public void setPositiveAngularRateLimit(double positiveAngularRateLimit) {
        this.positiveAngularRateLimit = positiveAngularRateLimit;
    }

    public void setPositiveLinearRateLimit(double positiveLinearRateLimit) {
        this.positiveLinearRateLimit = positiveLinearRateLimit;
    }

    public UnitlessChassisSpeeds calculate(UnitlessChassisSpeeds input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;

        double dx = input.x - prevSpeeds.x;
        double dSigX = Math.signum(prevSpeeds.x - input.x);
        double dy = input.y - prevSpeeds.y;
        double dSigY = Math.signum(prevSpeeds.y - input.y);
        double domega = input.omega - prevSpeeds.omega;
        double dSigOmega = Math.signum(prevSpeeds.omega - input.omega);

        prevSpeeds.x +=
        Math.signum(dx) * MathUtil.clamp(
            Math.abs(dx) * dSigX,
            negitiveLinearRateLimit * elapsedTime,
            positiveLinearRateLimit * elapsedTime
        );

        prevSpeeds.y +=
        Math.signum(dy) * MathUtil.clamp(
            Math.abs(dy) * dSigY,
            negitiveLinearRateLimit * elapsedTime,
            positiveLinearRateLimit * elapsedTime
        );

        prevSpeeds.omega +=
        Math.signum(domega) * MathUtil.clamp(
            Math.abs(domega) * dSigOmega,
            negitiveAngularRateLimit * elapsedTime,
            positiveAngularRateLimit * elapsedTime
        );

        
        prevTime = currentTime;
        return prevSpeeds;
    }

    public static class UnitlessChassisSpeeds {
        public double x, y, omega;
        public UnitlessChassisSpeeds (double x, double y, double omega) {
            this.x = x;
            this.y = y;
            this.omega = omega;
        }

        public UnitlessChassisSpeeds(ChassisSpeeds chassisSpeeds) {
            x = chassisSpeeds.vxMetersPerSecond;
            y = chassisSpeeds.vyMetersPerSecond;
            omega = chassisSpeeds.omegaRadiansPerSecond;
        }

        public UnitlessChassisSpeeds(ChassisPercentSpeeds chassisPercentSpeeds) {
            x = chassisPercentSpeeds.vxPercentOfMax;
            y = chassisPercentSpeeds.vyPercentOfMax;
            omega = chassisPercentSpeeds.omegaPercentOfMax;
        }

        public BreakerVector2 getLinearVelocityVector() {
            return new BreakerVector2(x, y);
        }

        public ChassisSpeeds getChassisSpeeds() {
            return new ChassisSpeeds(x, y, omega);
        }

        public ChassisPercentSpeeds getChassisPercentSpeeds() {
            return new ChassisPercentSpeeds(x, y, omega);
        }


    }
}
