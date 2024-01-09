// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// I SUGGEST USING OBLOG

package frc.robot.BreakerLib.driverstation.dashboard;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Wrapper for Shuffleboard dashboard. */
public class BreakerDashboard {
    private static List<ShuffleboardTab> allTabs = new ArrayList<>();

    /** @return Tab labeled "Main". */
    public static ShuffleboardTab getMainTab() {
        return getTab("Main");
    }

    /** @return Tab labeled "Setup". */
    public static ShuffleboardTab getSetupTab() {
        return getTab("Setup");
    }
    
    /** @return Tab labeled "Tuning". */
    public static ShuffleboardTab getTuningTab() {
        return getTab("Tuning");
    }

    /** @return Tab labeled "Diagnostics". */
    public static ShuffleboardTab getDiagnosticsTab() {
        return getTab("Diagnostics");
    }

    /** Creates/adds tab with given name.
     * 
     * @param tabName Name of tab
     * 
     * @return Tab.
     */
    public static ShuffleboardTab getTab(String tabName) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        if (!allTabs.contains(tab)) {
            allTabs.add(tab);
        }
        return tab;
    }

    /** @return All Shuffleboard tabs. */
    public static List<ShuffleboardTab> getAllTabs() {
        return allTabs;
    }
    
}
