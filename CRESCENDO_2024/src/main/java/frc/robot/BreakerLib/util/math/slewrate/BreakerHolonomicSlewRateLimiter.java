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
    private SlewRateLimiter omegaLimit, xLimit, yLimit;
    private double prevTime;
    public BreakerHolonomicSlewRateLimiter(double positiveLinearRateLimit, double negitiveLinearRateLimit, double positiveAngularRateLimit, double negitiveAngularRateLimit, UnitlessChassisSpeeds initalSpeeds) {
        prevTime = MathSharedStore.getTimestamp();
        prevSpeeds = initalSpeeds;
        omegaLimit = new SlewRateLimiter(positiveAngularRateLimit, negitiveAngularRateLimit, initalSpeeds.omega);
        xLimit = new SlewRateLimiter(positiveLinearRateLimit, negitiveLinearRateLimit, initalSpeeds.x);
        yLimit = new SlewRateLimiter(positiveLinearRateLimit, negitiveLinearRateLimit, initalSpeeds.y);
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
        // double currentTime = MathSharedStore.getTimestamp();
        // double elapsedTime = currentTime - prevTime;
        //calculateLinear(input, elapsedTime);
        // BreakerVector2 linearVels = calculateLinear(input, elapsedTime);
        // prevSpeeds.x = linearVels.getX();
        // prevSpeeds.y = linearVels.getY();

        double x = xLimit.calculate(input.x);
        if (input.x == 0.0) {
            x = 0.0;
            xLimit.reset(0.0);
        }

        double y = yLimit.calculate(input.y);
        if (input.y == 0.0) {
            y = 0.0;
            yLimit.reset(0.0);
        }
        
        double o = omegaLimit.calculate(input.omega);
        if (input.omega == 0.0) {
            o = 0.0;
            omegaLimit.reset(0.0);
        }
        // prevTime = currentTime;
        return new UnitlessChassisSpeeds(x, y, o);
    }

    // protected void calculateLinear(UnitlessChassisSpeeds input, double elapsedTime) {
    //     BreakerVector2 curVelVec = input.getLinearVelocityVector();
    //     BreakerVector2 posLimVec = new BreakerVector2(curVelVec.getVectorRotation(), Math.abs(positiveLinearRateLimit));
    //     BreakerVector2 negLimVec = new BreakerVector2(curVelVec.getVectorRotation(), Math.abs(negitiveLinearRateLimit));
    //     prevSpeeds.x += MathUtil.clamp(
    //             input.x - prevSpeeds.x,
    //             -Math.abs(negLimVec.getX()) * elapsedTime,
    //             Math.abs(posLimVec.getX()) * elapsedTime);
        
    //     prevSpeeds.y += MathUtil.clamp(
    //             input.y - prevSpeeds.y,
    //             -Math.abs(negLimVec.getY()) * elapsedTime,
    //             Math.abs(posLimVec.getY()) * elapsedTime);

    //             System.out.println(posLimVec);
    // }

    // protected BreakerVector2 calculateLinear(UnitlessChassisSpeeds input, double elapsedTime) {
        
          
    // }

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
