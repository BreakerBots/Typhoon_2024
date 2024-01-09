// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.brakemode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.util.BreakerRoboRIO;

/**
 * Automatically handles brake mode switching for your drivetrain based on the
 * robot's current mode and given config.
 */
public class BreakerAutoBrakeManager extends SubsystemBase {

    private boolean brakeInAuto;
    private boolean brakeInTeleop;
    private boolean brakeInTest;
    private boolean brakeInDisabled;
    private boolean autoBrakeIsEnabled = true;
    private BreakerGenericDrivetrain baseDrivetrain;

    /**
     * Creates an AutoBrake (automatic break mode) manager.
     * 
     * @param config Manager settings to use.
     */
    public BreakerAutoBrakeManager(BreakerAutoBrakeManagerConfig config) {
        changeConfig(config);
    }

    public void changeConfig(BreakerAutoBrakeManagerConfig config) {
        brakeInAuto = config.getBrakeInAuto();
        brakeInTeleop = config.getBrakeInTeleop();
        brakeInTest = config.getBrakeInTest();
        brakeInDisabled = config.getBrakeInDisabled();
        baseDrivetrain = config.getBaseDrivetrain();
    }

    /** @return If autobrake is enabled. */
    public boolean autoBrakeEnabled() {
        return autoBrakeIsEnabled;
    }

    /** Enables or disables autobrake. */
    public void setAutoBrakeEnabled(boolean isEnabled) {
        autoBrakeIsEnabled = isEnabled;
    }

    /** Updates brake mode based on current operating mode. */
    public void setAutoBrakeMode() {
        switch (BreakerRoboRIO.getCurrentRobotMode()) {
            case DISABLED:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInDisabled);
                break;
            case AUTONOMOUS:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInAuto);
                break;
            case TELEOP:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInTeleop);
                break;
            case TEST:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInTest);
                break;
            case UNKNOWN:
            default:
                baseDrivetrain.setDrivetrainBrakeMode(false);
                break;

        }
    }

    /** @return If robot enters brake mode in auto. */
    public boolean getBrakeInAuto() {
        return brakeInAuto;
    }

    /** @return If robot enters brake mode when disabled. */
    public boolean getBrakeInDisabled() {
        return brakeInDisabled;
    }

    /** @return If robot enters brake mode in teleop. */
    public boolean getBrakeInTeleop() {
        return brakeInTeleop;
    }

    /** @return If robot enters brake mode in test. */
    public boolean getBrakeInTest() {
        return brakeInTest;
    }

    @Override
    public void periodic() {
        if (BreakerRoboRIO.robotModeHasChanged()) {
            setAutoBrakeMode();
        }
    }
}
