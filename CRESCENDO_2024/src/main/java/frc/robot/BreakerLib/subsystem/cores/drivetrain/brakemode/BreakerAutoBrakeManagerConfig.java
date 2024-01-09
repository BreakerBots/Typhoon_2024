// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.brakemode;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;

/** Config for {@link BreakerAutoBrakeManager}. */
public class BreakerAutoBrakeManagerConfig {

    private boolean brakeInAuto;
    private boolean brakeInTeleop;
    private boolean brakeInTest;
    private boolean brakeInDisabled;
    private BreakerGenericDrivetrain baseDrivetrain;

    public BreakerAutoBrakeManagerConfig(BreakerGenericDrivetrain baseDrivetrain, boolean breakInTeleop,
            boolean breakInAuto, boolean breakInTest, boolean breakInDisabled) {
        this.brakeInAuto = breakInAuto;
        this.brakeInDisabled = breakInDisabled;
        this.brakeInTeleop = breakInTeleop;
        this.brakeInTest = breakInTest;
        this.baseDrivetrain = baseDrivetrain;
    }

    /**
     * Team 5104 default configuration, with all braking disabled along with the
     * robot for ease of transport but enabled otherwise.
     */
    public BreakerAutoBrakeManagerConfig(BreakerGenericDrivetrain baseDrivetrain) {
        this.brakeInAuto = true;
        this.brakeInDisabled = false;
        this.brakeInTeleop = true;
        this.brakeInTest = true;
        this.baseDrivetrain = baseDrivetrain;
    }

    public boolean getBrakeInAuto() {
        return brakeInAuto;
    }

    public boolean getBrakeInDisabled() {
        return brakeInDisabled;
    }

    public boolean getBrakeInTeleop() {
        return brakeInTeleop;
    }

    public boolean getBrakeInTest() {
        return brakeInTest;
    }

    public BreakerGenericDrivetrain getBaseDrivetrain() {
        return baseDrivetrain;
    }
}
