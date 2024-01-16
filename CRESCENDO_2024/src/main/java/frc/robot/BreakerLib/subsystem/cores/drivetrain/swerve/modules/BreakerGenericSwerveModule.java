// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import org.littletonrobotics.junction.LogTable;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLogUtil;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Interface for all Swerve Modules to allow for easy interchangeablity, this
 * class is meant to serve as an intermedairy between your swerve hardware and
 * the BreakerSwerveDrive class
 */
public abstract class BreakerGenericSwerveModule extends BreakerGenericDevice implements Sendable {
    protected Translation2d wheelPositionRelativeToRobot;
    public BreakerGenericSwerveModule(Translation2d wheelPositionRelativeToRobot) {
        this.wheelPositionRelativeToRobot = wheelPositionRelativeToRobot;
    }

    /**
     * default method for setting a swerve module to a given target state,
     * automaticly calls the overloded version of this method that independently
     * specifyes angle and speed
     */
    public void setModuleTarget(SwerveModuleState targetModuleState, boolean isOpenLoop) {
        setModuleTarget(targetModuleState.angle, targetModuleState.speedMetersPerSecond, isOpenLoop);
    }

    /** Sets the modules target speed to zero while maintaning the last set angle */
    public void stop() {
        setModuleTarget(getModuleTargetState().angle, 0.0, false);
    }

    /**
     * Method defined in module code to handle angle and velocity control of the
     * module
     */
    public abstract void setModuleTarget(Rotation2d targetAngle, double targetVelocityMetersPerSecond, boolean isOpenLoop);

    /** @return the absolute (+/- 180 deg) angle of the module as a {@link Rotation2d} */
    public abstract Rotation2d getModuleAbsoluteAngle();

    /**
     * @return the relative (with rollover, 180 -> 181) angle of the module in as a {@link Rotation2d}
     */
    public abstract Rotation2d getModuleRelativeAngle();

    /** @return The velocity of the module's drive wheel in meters per second. */
    public abstract double getModuleVelMetersPerSec();

    public abstract SwerveModuleState getModuleTargetState();

    /** @return Module's {@link SwerveModuleState}. */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleVelMetersPerSec(), getModuleAbsoluteAngle());
    }

    /**
     * Sets brake mode on drive motor.
     * 
     * @param isEnabled Whether or not to enable brake mode.
     */
    public abstract void setDriveMotorBrakeMode(boolean isEnabled);

    /**
     * Sets brake mode on turn motor.
     * 
     * @param isEnabled Whether or not to enable brake mode.
     */
    public abstract void setTurnMotorBrakeMode(boolean isEnabled);

    /**
     * Sets brake mode on drive and turn motors.
     * 
     * @param isEnabled Whether or not to enable brake mode.
     */
    public abstract void setModuleBrakeMode(boolean isEnabled);

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getModuleDriveDistanceMeters(), getModuleAbsoluteAngle());
    }

    public abstract double getModuleDriveDistanceMeters();

    public abstract void resetModuleDriveEncoderPosition();

    public abstract double getMaxAttainableWheelSpeed();

    public abstract void setStatusUpdatePeriod(double period);

    /**
     * @return Module's health as an array.
     *         <p>
     *         [0] = overall, [1] = drive motor, [2] = turn motor, [3... n] = other devices if
     *         supported (EX: CANCoder)
     */
    public abstract DeviceHealth[] getModuleHealths();

    public static String getModuleAsString(String moduleType, BreakerGenericSwerveModule module) {
        return String.format("%s(Name: %s, Overall_Device_Health: %s, Set_State: %s, Current_State: %s)", moduleType, module.getDeviceName(), module.getHealth().toString(), module.getModuleTargetState().toString(), module.getModuleState().toString());
    }

    public Translation2d getWheelPositionRelativeToRobot() {
        return wheelPositionRelativeToRobot;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Velocity", () -> this.getModuleTargetState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Target Angle", () -> this.getModuleTargetState().angle.getDegrees(), null); 
        builder.addDoubleProperty("Actual Velocity", () -> this.getModuleVelMetersPerSec(), null);
        builder.addDoubleProperty("Actual Angle", () -> this.getModuleAbsoluteAngle().getDegrees(), null); 
    }

    @Override
    public void toLog(LogTable table) {
        table.put("ModuleState", BreakerLogUtil.formatSwerveModuleStateForLog(getModuleState()));
        table.put("WheelDistanceMeters", getModuleDriveDistanceMeters());
        table.put("OverallHealth", getHealth().toString());
    }
}
