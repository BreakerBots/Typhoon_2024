// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Config class for {@link BreakerLegacySwerveDrive}. */
public class BreakerSwerveDriveConfig {

    private double maxLinearVel;
    private double maxAngleVel;
    private double slowModeLinearMultiplier;
    private double slowModeTurnMultiplier;
    private double moduleWheelSpeedDeadband;
    private Optional<PIDController> headingCompensationController;
    private Optional<Double> headingCompensationAngularVelDeadband, headingCompensationMinActiveLinearSpeed;

    /**
     * The overall configuration for a Breaker swerve drivetrain holding all constants,
     * must be passed in.
     * 
     * @param maxForwardVel                  Max forward strafe velocity of drivetrain in
     *                                       m/s.
     * @param maxSidewaysVel                 Max horizontal strafe velocity of drivetrain in
     *                                       m/s.
     * @param maxAngVel                      Max angular velocity of the drivetrain
     *                                       in rad/s.
     * @param wheelspeedDeadband             The min value (+/-) in m/s^2 that each modules wheel speed can be set too before being ingored
     * @param maxAttainableModuleWheelSpeed  The physical maximum speed (in m/s^2) your swerve modules are capable of achiving
     */
    public BreakerSwerveDriveConfig(double maxLinearVel, double maxAngVel, double moduleWheelSpeedDeadband) {

        this.maxLinearVel = maxLinearVel;
        this.maxAngleVel = maxAngVel;
        this.moduleWheelSpeedDeadband = moduleWheelSpeedDeadband;
        slowModeLinearMultiplier = 1;
        slowModeTurnMultiplier = 1;
    }

    public BreakerSwerveDriveConfig withHeadingCompensation(double headingCompensationAngularVelDeadband, double headingCompensationMinActiveLinearSpeed, PIDController headingCompensationController) {
        this.headingCompensationAngularVelDeadband = Optional.of(headingCompensationAngularVelDeadband);
        this.headingCompensationMinActiveLinearSpeed = Optional.of(headingCompensationMinActiveLinearSpeed);
        headingCompensationController.enableContinuousInput(-Math.PI, Math.PI);
        this.headingCompensationController = Optional.of(headingCompensationController);
        return this;
    
    }

    public BreakerSwerveDriveConfig withoutHeadingCompensation() {
        headingCompensationAngularVelDeadband = Optional.empty();
        headingCompensationMinActiveLinearSpeed = Optional.empty();
        headingCompensationController = Optional.empty();
        return this;
    }

    /**
     * Sets slow mode multipliers.
     * 
     * @param linearMulitplier Slow mode multiplier for drive motors.
     * @param turnMultiplier Slow mode multiplier for turn motors.
     */
    public BreakerSwerveDriveConfig withSlowModeMultipliers(double linearMulitplier, double turnMultiplier) {
        slowModeLinearMultiplier = linearMulitplier;
        slowModeTurnMultiplier = turnMultiplier;
        return this;
    }

    /** @return Max linear velocity magnitude of swerve drive in m/s. Usually the same as max sideways velocity. */
    public double getMaxLinearVel() {
        return maxLinearVel;
    }

    /** @return Max angular velocity of swerve drive in rad/s.*/
    public double getMaxAngleVel() {
        return maxAngleVel;
    }

    /** @return Slow mode multiplier on drive motors. */
    public double getSlowModeLinearMultiplier() {
        return slowModeLinearMultiplier;
    }

    /** @return Slow mode multiplier on turn motors. */
    public double getSlowModeTurnMultiplier() {
        return slowModeTurnMultiplier;
    }

    public double getModuleWheelSpeedDeadband() {
        return moduleWheelSpeedDeadband;
    }

    public Optional<Double> getHeadingCompensationAngularVelDeadband() {
        return headingCompensationAngularVelDeadband;
    }

    public Optional<Double> getHeadingCompensationMinActiveLinearSpeed() {
        return headingCompensationMinActiveLinearSpeed;
    }

    public Optional<PIDController> getHeadingCompensationController() {
        return headingCompensationController;
    }
}
