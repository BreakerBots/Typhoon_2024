// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoManager;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.brakemode.BreakerAutoBrakeManager;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.brakemode.BreakerAutoBrakeManagerConfig;

/**
 * Robot manager that configures SelfTest functionality, automatic brake mode,
 * and auto paths.
 */
public class BreakerRobotManager {
    private static SelfTest test;
    private static BreakerAutoManager autoManager;
    private static BreakerAutoBrakeManager brakeModeManager;
    private static BreakerGenericDrivetrain baseDrivetrain;

    private BreakerRobotManager() {
    }

    /** Setup for the BreakerRobotManager.
     * 
     * @param baseDrivetrain Base drivetrain.
     * @param robotConfig Robot configuration.
     */
    public static void setup(BreakerGenericDrivetrain baseDrivetrain, BreakerRobotConfig robotConfig) {
    
        // Set up data receivers & replay source
        if (RobotBase.isReal()) {
            BreakerLog.addDataReceiver(new WPILOGWriter(robotConfig.getRealLogFilePath()));
            BreakerLog.addDataReceiver(new NT4Publisher());
        } else {
            BreakerLog.addDataReceiver(new WPILOGWriter(robotConfig.getSimLogFilePath()));
            BreakerLog.addDataReceiver(new NT4Publisher());
        }
        test = new SelfTest(robotConfig.getSecondsBetweenSelfChecks(),
                    robotConfig.getAutoRegisterDevices());
        BreakerRobotManager.baseDrivetrain = baseDrivetrain;
        BreakerRobotManager.autoManager = robotConfig.usesPaths() ? new BreakerAutoManager(robotConfig.getAutoPaths())
                : new BreakerAutoManager();
        BreakerRobotManager.brakeModeManager = new BreakerAutoBrakeManager(
                new BreakerAutoBrakeManagerConfig(baseDrivetrain));
        BreakerLog.start(robotConfig.getStartConfig());
    }

    /** @return Brake mode manager object. */
    public static BreakerAutoBrakeManager getBrakeModeManager() {
        return brakeModeManager;
    }

    /** @return SelfTest object. */
    public static SelfTest getSelfTest() {
        return test;
    }

    /** @return Auto manager object. */
    public static BreakerAutoManager getAutoManager() {
        return autoManager;
    }

    /** @return Autopath selected through auto manager. */
    public static Command getSelectedAutoPath() {
        return autoManager.getSelectedAutoPath();
    }

    /** Enable or disable brake mode. */
    public static void setDrivetrainBrakeMode(boolean isEnabled) {
        baseDrivetrain.setDrivetrainBrakeMode(isEnabled);
    }
}
