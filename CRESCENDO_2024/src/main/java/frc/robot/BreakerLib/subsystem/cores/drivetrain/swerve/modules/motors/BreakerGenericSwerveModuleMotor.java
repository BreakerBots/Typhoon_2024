// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors;

import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLoggable;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

/** Add your docs here. */
public abstract class BreakerGenericSwerveModuleMotor extends BreakerSelfTestableBase implements BreakerLoggable {
    public abstract void setBrakeMode(boolean isEnabled);
    public abstract double getSupplyCurrent();
    public abstract double getMotorOutput();
    @Override
    public void toLog(LogTable table) {
        table.put("SupplyCurrentAmps", getSupplyCurrent());
        table.put("MotorOutput", getMotorOutput());
        table.put("DeviceHealth", getHealth().toString());
    }

}
