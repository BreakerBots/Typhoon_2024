// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.slewrate;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest.ChassisPercentSpeeds;

/** Add your docs here. */
public class BreakerHolonomicSlewRateLimiter {
    private double positiveLinearRateLimit, negitiveLinearRateLimit, positiveAngularRateLimit, negitiveAngularRateLimit;
    private UnitlessChassisSpeeds prevSpeeds;
    private double prevTime;
    public BreakerHolonomicSlewRateLimiter(double positiveLinearRateLimit,  double negitiveLinearRateLimit, double positiveAngularRateLimit, double negitiveAngularRateLimit, UnitlessChassisSpeeds initalSpeeds) {
        prevTime = MathSharedStore.getTimestamp();
        prevSpeeds = initalSpeeds;
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
        calculateLinear(input, elapsedTime);
        prevSpeeds.omega +=
            MathUtil.clamp(
                input.omega - prevSpeeds.omega,
                negitiveAngularRateLimit * elapsedTime,
                positiveAngularRateLimit * elapsedTime);
        prevTime = currentTime;
        return prevSpeeds;
    }

    protected void calculateLinear(UnitlessChassisSpeeds input, double elapsedTime) {
        BreakerVector2 curVelVec = input.getLinearVelocityVector();
        BreakerVector2 posLimVec = new BreakerVector2(curVelVec.getVectorRotation(), positiveLinearRateLimit);
        BreakerVector2 negLimVec = new BreakerVector2(curVelVec.getVectorRotation(), negitiveLinearRateLimit);
        prevSpeeds.x += MathUtil.clamp(
                input.x - prevSpeeds.x,
                negLimVec.getX() * elapsedTime,
                posLimVec.getX() * elapsedTime);
        
        prevSpeeds.y += MathUtil.clamp(
                input.y - prevSpeeds.y,
                negLimVec.getY() * elapsedTime,
                posLimVec.getY() * elapsedTime);
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
