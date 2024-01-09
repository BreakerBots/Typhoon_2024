// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.bindings;

/** Add your docs here. */
public class BreakerButtonBindingManager {
    private static int indexOfMapToUse = 0;
    private static int nextMapIndex = 0;
    
    /** 
     * @param mapToAdd
     */
    public static void addButtonBindingMap(BreakerButtonBindingMap mapToAdd) {
        final int index = nextMapIndex;
        mapToAdd.bindAllConditionaly(() -> indexOfMapToUse == index);
        nextMapIndex++;
    }

    
    /** 
     * @return int
     */
    public static int getActiveButtonBindingMapIndex() {
        return indexOfMapToUse;
    }

    
    /** 
     * @param indexOfMapToUse
     */
    public static void setActiveButtonBindingMap(int indexOfMapToUse) {
        BreakerButtonBindingManager.indexOfMapToUse = indexOfMapToUse;
    }
}
