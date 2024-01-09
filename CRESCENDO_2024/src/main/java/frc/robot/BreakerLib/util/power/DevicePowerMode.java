// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.power;

/** A class representing the power conservation mode of a device */
public enum DevicePowerMode {
    /** device is 100% operational and at full power draw */
    FULL_POWER_MODE,
    /** device has made some cutbacks to conserve power but is mostly still nominal */
    MEDIUM_POWER_MODE,
    /** device has made extream cutbacks to conserve as much power as possible without being deactivated */
    LOW_POWER_MODE,
    /** device has been soft deactivated to conserve power */
    HIBERNATEING
}
