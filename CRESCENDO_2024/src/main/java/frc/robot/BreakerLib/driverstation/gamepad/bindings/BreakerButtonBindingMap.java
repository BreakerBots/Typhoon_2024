// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.bindings;

import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class BreakerButtonBindingMap {
    private BreakerButtonBinding[] buttonBindings;
    public BreakerButtonBindingMap(BreakerButtonBinding... buttonBindings) {
        this.buttonBindings = buttonBindings;
    }

    
    /** 
     * @return BreakerButtonBinding[]
     */
    public BreakerButtonBinding[] getButtonBindings() {
        return buttonBindings;
    }

    public void bindAll() {
        for (BreakerButtonBinding bind: buttonBindings) {
            bind.bind();
        }
    }

    
    /** 
     * @param isBindingEnabledSupplier
     */
    public void bindAllConditionaly(BooleanSupplier isBindingEnabledSupplier) {
        for (BreakerButtonBinding bind: buttonBindings) {
            bind.bindConditionaly(isBindingEnabledSupplier);
        }
    }
}
