// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.BreakerLib.auto.BreakerAutoPath;
import frc.robot.BreakerLib.util.power.BreakerPowerDistribution;

/** Config class for the robot, used by {@link BreakerRobotManager}. */
public class BreakerRobotConfig {

    private double secondsBetweenSelfChecks;
    private boolean autoRegisterDevices;
    private boolean useBrakeModeManager;
    private BreakerAutoPath[] autoPaths;
    private boolean usesPaths;
    private BreakerRobotStartConfig startConfig;
    private String realLogPath, simLogPath;

    /**
     * Default robot configuration. Pre-assigned settings are listed below:
     * <p>
     * - Seconds between self-checks = 5.
     * <p>
     * - Devices are automatically registered into SelfTest.
     * <p>
     * - Brake mode manager is enabled.
     * <p>
     * - NetworkTables aren't logged.
     * <p>
     * - Auto paths are not used.
     * <p>
     * - Falcon Orchestra is not used.
     * 
     * @param startConfig Startup configuration to use.
     */
    public BreakerRobotConfig(BreakerRobotStartConfig startConfig) {
        this.startConfig = startConfig;

        this.secondsBetweenSelfChecks = 5;
        this.autoRegisterDevices = true;
        this.useBrakeModeManager = true;
        this.simLogPath = "";
        this.realLogPath = "";
        this.autoPaths = new BreakerAutoPath[0];
        usesPaths = false;
    }

    public void setPowerDistributionModule(int module, PowerDistribution.ModuleType moduleType) {
        BreakerPowerDistribution.getInstance(module, moduleType);
    }

    //"/media/sda1/"
    public void setLogFilePaths(String realLogPath, String simLogPath) {
        this.realLogPath = realLogPath;
        this.simLogPath = simLogPath;
    }

    /**
     * Allows for selection of auto paths on Shuffleboard GUI.
     * 
     * @param paths Autopaths to select from.
     */
    public void setAutoPaths(BreakerAutoPath... paths) {
        autoPaths = paths;
        usesPaths = true;
    }

    /**
     * Enables a message printed to the console on startup with robot info.
     * 
     * @param startConfig Robot info to use.
     */
    public void setStartConfig(BreakerRobotStartConfig startConfig) {
        this.startConfig = startConfig;
    }

    /**
     * Sets whether or not to automatically register compatible devices into
     * BreakerLib's Self-Test functionality.
     * 
     * @param enabled True for enable, false for disable.
     */
    public void setAutoRegisterDevices(boolean enabled) {
        autoRegisterDevices = enabled;
    }

    /**
     * Sets whether or not to enable BrakeModeManager.
     * 
     * @param enabled True for enable, false for disable.
     */
    public void setBrakeModeManagerEnabled(boolean enabled) {
        useBrakeModeManager = enabled;
    }

    /**
     * Sets the number of seconds until another Self-Test is run.
     * 
     * @param seconds Seconds to wait.
     */
    public void setSecondsBetweenSelfTests(double seconds) {
        this.secondsBetweenSelfChecks = seconds;
    }

    /** @return Auto-register device setting. */
    public boolean getAutoRegisterDevices() {
        return autoRegisterDevices;
    }

    /** @return Array of auto paths. */
    public BreakerAutoPath[] getAutoPaths() {
        return autoPaths;
    }

    /** @return Brake mode manager enabled setting.*/
    public boolean getBrakeModeManagerEnabled() {
        return useBrakeModeManager;
    }

    /** @return Number of seconds between self-checks. */
    public double getSecondsBetweenSelfChecks() {
        return secondsBetweenSelfChecks;
    }

    /** @return If auto paths are enabled. */
    public boolean usesPaths() {
        return usesPaths;
    }

    /** @return Robot start config. */
    public BreakerRobotStartConfig getStartConfig() {
        return startConfig;
    }

    public String getRealLogFilePath() {
        return realLogPath;
    }

    public String getSimLogFilePath() {
        return simLogPath;
    }

}
