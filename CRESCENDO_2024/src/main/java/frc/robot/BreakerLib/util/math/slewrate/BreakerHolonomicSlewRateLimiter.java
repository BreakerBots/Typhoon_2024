// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.slewrate;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwervePercentSpeedRequest.ChassisPercentSpeeds;
import frc.robot.BreakerLib.util.BreakerTriplet;

/** Add your docs here. */
public class BreakerHolonomicSlewRateLimiter {
    private double positiveLinearRateLimit, negitiveLinearRateLimit, positiveAngularRateLimit, negitiveAngularRateLimit;
    private UnitlessChassisSpeeds prevSpeeds;
    private double prevTime;
    public BreakerHolonomicSlewRateLimiter(double positiveLinearRateLimit,  double negitiveLinearRateLimit, double positiveAngularRateLimit, double negitiveAngularRateLimit, UnitlessChassisSpeeds initalSpeeds) {
        prevTime = MathSharedStore.getTimestamp();
        this.positiveLinearRateLimit = positiveLinearRateLimit;
        this.negitiveLinearRateLimit = negitiveLinearRateLimit;
        this.positiveAngularRateLimit = positiveAngularRateLimit;
        this.negitiveAngularRateLimit = negitiveAngularRateLimit;
        prevSpeeds = initalSpeeds;
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
        BreakerVector2 linearVels = calculateLinear(input, elapsedTime);
        prevSpeeds.x = linearVels.getX();
        prevSpeeds.y = linearVels.getY();

        prevSpeeds.omega +=
            MathUtil.clamp(
                input.omega - prevSpeeds.omega,
                negitiveAngularRateLimit * elapsedTime,
                positiveAngularRateLimit * elapsedTime);
        prevTime = currentTime;
        return prevSpeeds;
    }

    protected BreakerVector2 calculateLinear(UnitlessChassisSpeeds input, double elapsedTime) {
        BreakerVector2 prevVelVec = prevSpeeds.getLinearVelocityVector();
        BreakerVector2 deltaVec = input.getLinearVelocityVector().minus(prevVelVec);
        double clampedDeltaMag = MathUtil.clamp(deltaVec.getMagnitude(), negitiveLinearRateLimit * elapsedTime, positiveLinearRateLimit * elapsedTime);
        BreakerVector2 clampedDeltaVec = new BreakerVector2(deltaVec.getVectorRotation(), clampedDeltaMag);
        System.out.println(deltaVec.getVectorRotation());
        return prevVelVec.plus(clampedDeltaVec);
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
            return new ChassisPercentSpeeds(x, x, omega);
        }


    }
}
