// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors.binary;

import frc.robot.BreakerLib.devices.BreakerGenericDevice;

/** Add your docs here. */
public abstract class BreakerGenericBinaryMotor extends BreakerGenericDevice {
      
        /** Sets motor to designated percent output. */
        public abstract void start();
    
        /** Sets motor to 0% output (stopped) */
        public abstract void stop();
        /** Checks if motor is running or not. */
        public abstract boolean isActive();
    
        public void toggle() {
            if (isActive()) {
                stop();
            } else {
                start();
            }
        }
}
