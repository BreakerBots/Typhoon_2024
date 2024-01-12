package frc.robot.BreakerLib.util.math;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.lang.model.type.MirroredTypeException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BreakerLib.physics.Breaker3AxisForces;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;

/** BreakerLib math util class. */
public class BreakerMath {

    private static double prevTime = 0;

    /**
     * Constrains an angle value in degrees within +- 180 degrees.
     * 
     * @param deg Angle value in degrees.
     * 
     * @return Angle value within -180 to +180 degrees.
     */
    public static final double angleModulus(double deg) {
        return angleModulus(deg, 180);
    }

    /**
     * Constrains an angle value in degrees within +- desired constraint, in degrees
     * 
     * @param deg        Angle value in degrees.
     * @param constraint Degree value to constrain angle within. Should be > 0.
     * 
     * @return Angle value within -constraint and +constraint degrees.
     */
    public static final double angleModulus(double deg, double constraint) {
        return MathUtil.inputModulus(deg, -constraint, constraint);
    }

    /**
     * Constrains an angle value in degrees within a minimum and maximum angle.
     * 
     * @param deg      Angle value in degrees.
     * @param minAngle Minimum angle value.
     * @param maxAngle Maximum angle value.
     * 
     * @return Angle value between minAngle and maxAngle.
     */
    public static final double angleModulus(double deg, double minAngle, double maxAngle) {
        return MathUtil.inputModulus(deg, minAngle, maxAngle);
    }

    /** @return Radians per second from rotations per minute. */
    public static final double rpmToRadPerSec(double rpm) {
        return ((rpm / 60) * (2 * Math.PI));
    }

    /** @return Rotations per minute from radians per second. */
    public static final double radPerSecToRPM(double radPerSec) {
        return ((radPerSec * 60) / (2 * Math.PI));
    }

    /**
     * @return Time difference between cycles, in seconds.
     */
    public static double getCycleDiffTime() {
        double curTime = RobotController.getFPGATime(); // In microseconds
        double diffTime = curTime - prevTime;
        prevTime = curTime;
        diffTime = BreakerUnits.microsecondsToSeconds(diffTime); // Value converted to seconds
        return diffTime;
    }

    public static double getCircumferenceFromDiameter(double diameter) {
        return diameter * Math.PI;
    }

    public static double getTicksPerRotation(double encoderTicks, double gearRatioTo1) {
        return encoderTicks * gearRatioTo1;
    }

    public static double getTicksPerInch(double encoderTicks, double gearRatioTo1, double wheelDiameter) {
        return getTicksPerRotation(encoderTicks, gearRatioTo1) / getCircumferenceFromDiameter(wheelDiameter);
    }

    /**
     * @param ticks Talon FX encoder ticks.
     * @return distance, in inches.
     */
    public static double ticksToInches(double ticks, double ticksPerInch) {
        return ticks / ticksPerInch;
    }

    public static double getAvg(double lastAvg, double newVal, int cycleCount) {
        return (((lastAvg * (cycleCount - 1)) + newVal) / cycleCount);
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(int FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         long integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(Long FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         short integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(Short FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    /**
     * @param encoderTicks Ticks per rotation.
     * @return Rotational position in radians.
     */
    public static double radiansPerTick(double encoderTicks) {
        return (2.0 * Math.PI / encoderTicks);
    }

    // /**
    //  * Checks if two numbers are sufficiently proximate.
    //  * 
    //  * @param val1         First number.
    //  * @param val2         Second number.
    //  * @param maxDeviation Absolute value difference between val1 and val2
    //  * 
    //  * @return true if within deviation, false otherwise.
    //  */
    // public static boolean epsilonEquals(double val1, double val2, double maxDeviation) {
    //     return ((val1 <= (val2 + maxDeviation)) && (val1 >= (val2 - maxDeviation)));
    //     MathUtil.isNear()
    // }

    public static Rotation2d getPointAngleRelativeToOtherPoint(Translation2d point1, Translation2d point2) {
        double x1 = point1.getX();
        double y1 = point1.getY();
        double x2 = point2.getX();
        double y2 = point2.getY();
        if (x2 >= x1) {
            return Rotation2d.fromRadians(Math.atan2(y2 - y1, x2 - x1));
        } else {
            if (y2 >= y1) {
                double a = Math.atan2(y2 - y1, x2 - x1);
                a *= -Math.signum(a);
                return Rotation2d.fromRadians(Math.PI - a);
            } else {
                double a = Math.atan2(y2 - y1, x2 - x1);
                a *= -Math.signum(a);
                return Rotation2d.fromRadians(-Math.PI + a);
            }
        }
    }

    /**
     * Linearly interpolates between 2 points to find y-val at given x.
     * 
     * @param queryX X-value to interpolate from.
     * @param lowX   X-val of low point.
     * @param highX  X-val of high point.
     * @param lowY   Y-val of low point.
     * @param highY  Y-val of high point.
     * @return Approximate Y-value at given X.
     */
    public static double interpolateLinear(double queryX, double lowX, double highX, double lowY, double highY) {
        return MathUtil.interpolate(lowY, highY, getLerpT(queryX, lowX, highX));
    }

    public static double getLerpT(double query, double low, double high) {
        return (query - low) / (high - low);
    }

    /**
     * Lagrange Polynomial interpolation of a Y value from an X value and a set of
     * known points. https://en.wikipedia.org/wiki/Lagrange_polynomial
     * 
     * @param queryX      X-value to interpolate a Y-value for.
     * @param knownPoints Known points in a 2D space.
     * @return The approximate Y-Value that would corespond to the given X-Value
     */
    public static double interpolateLagrange(double queryX, Translation2d... knownPoints) {
        double result = 0;
        for (int i = 0; i < knownPoints.length; i++) { // Goes through points.
            double term = knownPoints[i].getY(); // Y-value of selected point.
            for (int j = 0; j < knownPoints.length; j++) { // Loops through non-identical points.
                if (j != i) { // Avoids multiplication by 0.
                    // Interpolates between selected and point from data set.
                    term *= (queryX - knownPoints[j].getX()) / (knownPoints[i].getX() - knownPoints[j].getX());
                }
            }
            result += term; // Accumulated interpretation is added.
        }
        return result;
    }

    public static BreakerMovementState2d movementStateFromChassisSpeedsAndPreviousState(Pose2d currentPose,
            ChassisSpeeds speeds, double timeToLastUpdateMiliseconds, BreakerMovementState2d prevMovementState) {
        Breaker3AxisForces acceleration = new Breaker3AxisForces(
                new BreakerVector2(
                        (1000 / timeToLastUpdateMiliseconds) * (speeds.vxMetersPerSecond
                                - prevMovementState.getDerivativefromIndex(0).getLinearForces().getX()),
                        (1000 / timeToLastUpdateMiliseconds) * (speeds.vyMetersPerSecond
                                - prevMovementState.getDerivativefromIndex(0).getLinearForces().getY())),
                (1000 / timeToLastUpdateMiliseconds)
                        * (speeds.omegaRadiansPerSecond
                                - prevMovementState.getDerivativefromIndex(0).getAngularForce()));
        Breaker3AxisForces jerk = new Breaker3AxisForces(
                new BreakerVector2(
                        (1000 / timeToLastUpdateMiliseconds) * (acceleration.getLinearForces().getX()
                                - prevMovementState.getDerivativefromIndex(1).getLinearForces().getY()),
                        (1000 / timeToLastUpdateMiliseconds) * (acceleration.getLinearForces().getY()
                                - prevMovementState.getDerivativefromIndex(1).getLinearForces().getY())),
                (1000 / timeToLastUpdateMiliseconds) * (acceleration.getAngularForce()
                        - prevMovementState.getDerivativefromIndex(1).getAngularForce()));
        return new BreakerMovementState2d(currentPose,
                new Breaker3AxisForces(new BreakerVector2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                        speeds.omegaRadiansPerSecond),
                acceleration, jerk);
    }

    public static ChassisSpeeds fromRobotRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds, Rotation2d robotAngle) {
        double cos = Math.cos(-robotAngle.getRadians());
        double sin = Math.sin(-robotAngle.getRadians());
        return new ChassisSpeeds(
                (robotRelativeSpeeds.vxMetersPerSecond * cos) - (robotRelativeSpeeds.vyMetersPerSecond * sin),
                (robotRelativeSpeeds.vxMetersPerSecond * sin) + (robotRelativeSpeeds.vyMetersPerSecond * cos),
                robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    public static double getWeightedAvg(double[] valuesToAvg, double[] weights) {
        double numer = 0;
        double denom = 0;
        // Values multiplied by respective weights.
        for (int i = 0; i < valuesToAvg.length; i++) {
            double weight = i < weights.length ? weights[i] : 1.0;
            numer += valuesToAvg[i] * weight;
        }
        // Weights added up.
        for (int i = 0; i < valuesToAvg.length; i++) {
            denom += i < weights.length ? weights[i] : 1.0;
        }

        return numer / denom;
    }

    public static double getWeightedAvg(List<Double> valuesToAvg, List<Double> weights) {
        double numer = 0;
        double denom = 0;
        for (int i = 0; i < valuesToAvg.size(); i++) {
            double weight = i < weights.size() ? weights.get(i) : 1.0;
            numer += valuesToAvg.get(i) * weight;
        }

        for (int i = 0; i < valuesToAvg.size(); i++) {
            denom += i < weights.size() ? weights.get(i) : 1.0;
        }

        return numer / denom;
    }

    public static double root(double num, double root) {
        return Math.pow(num, 1.0 / root);
    }

    public static double absoluteAngleToContinuousRelativeAngleDegrees(double curRelativeAngle,
            Rotation2d curAbsoluteAngle, Rotation2d tgtAngle) {
        return curRelativeAngle + (tgtAngle.minus(curAbsoluteAngle).getDegrees());
    }

    public static Pose2d mirrorPose(Pose2d pose, double translationalAxisOfSymetry, MirrorSymetryAxis2d translationMirrorType, MirrorSymetryAxis2d rotationMirrorType) {
       return new Pose2d(mirrorTranslation(pose.getTranslation(), translationalAxisOfSymetry, translationMirrorType), mirrorRotation(pose.getRotation(), rotationMirrorType));
    }

    public static enum MirrorSymetryAxis2d {
        X,
        Y,
        X_AND_Y
    } 

    public static Translation2d mirrorTranslation(Translation2d translation, double axisOfSymetry, MirrorSymetryAxis2d mirrorType) {
        if (mirrorType == MirrorSymetryAxis2d.Y) {
            double distance = axisOfSymetry - translation.getX();
            return new Translation2d(axisOfSymetry + distance, translation.getY());
        } else if (mirrorType == MirrorSymetryAxis2d.X) {
            double distance = axisOfSymetry - translation.getY();
            return new Translation2d(translation.getX(), axisOfSymetry + distance);
        }
        double distanceX = axisOfSymetry - translation.getX();
        double distanceY = axisOfSymetry - translation.getY();
        return new Translation2d(axisOfSymetry + distanceX, axisOfSymetry + distanceY);
    }

    public static Rotation2d mirrorRotation(Rotation2d angle, MirrorSymetryAxis2d mirrorType) {
        if (mirrorType == MirrorSymetryAxis2d.Y) {
            return new Rotation2d(-angle.getCos(), angle.getSin());
        } else if (mirrorType == MirrorSymetryAxis2d.X) {
            return new Rotation2d(angle.getCos(), -angle.getSin());
        }
        return new Rotation2d(-angle.getCos(), -angle.getSin());
        
    }

    public static boolean epsilonEqualsPose2d(Pose2d pose0, Pose2d pose1, Pose2d maxDeviation) {
        return MathUtil.isNear(pose0.getX(), pose1.getX(), maxDeviation.getX()) && 
                MathUtil.isNear(pose0.getY(), pose1.getY(), maxDeviation.getY()) && 
                MathUtil.isNear(pose0.getRotation().getRadians(), pose1.getRotation().getRadians(), maxDeviation.getRotation().getRadians());
    }

    public static boolean epsilonEqualsChassisSpeeds(ChassisSpeeds chassisSpeeds0, ChassisSpeeds chassisSpeeds1, ChassisSpeeds maxDeviation) {
        return MathUtil.isNear(chassisSpeeds0.vxMetersPerSecond, chassisSpeeds1.vxMetersPerSecond, maxDeviation.vxMetersPerSecond) && 
                MathUtil.isNear(chassisSpeeds0.vyMetersPerSecond, chassisSpeeds1.vyMetersPerSecond, maxDeviation.vyMetersPerSecond) && 
                MathUtil.isNear(chassisSpeeds0.omegaRadiansPerSecond, chassisSpeeds1.omegaRadiansPerSecond, maxDeviation.omegaRadiansPerSecond);
    }

    public static ChassisSpeeds clampChassisSpeeds(ChassisSpeeds speedsToClamp, double maxLinearVel, double maxAngularVel) {
        BreakerVector2 linVelVec = new BreakerVector2(speedsToClamp.vxMetersPerSecond, speedsToClamp.vyMetersPerSecond).clampMagnitude(0.0, maxLinearVel);
        return new ChassisSpeeds(linVelVec.getX(), linVelVec.getY(), MathUtil.clamp(speedsToClamp.omegaRadiansPerSecond, -maxAngularVel, maxAngularVel));
    }

}
