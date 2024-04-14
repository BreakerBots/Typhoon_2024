// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.newSwerve.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;

/** Add your docs here. */
public class BreakerSwerveMotorCurrentLimitConfig {
    private CurrentLimitsConfigs limitsConfig;
    private TorqueCurrentConfigs torqueCurrentConfig;
    public BreakerSwerveMotorCurrentLimitConfig(CurrentLimitsConfigs limitConfig) {
        this.limitsConfig = limitConfig;
        torqueCurrentConfig = new TorqueCurrentConfigs();
        torqueCurrentConfig.PeakForwardTorqueCurrent = Math.min(limitConfig.SupplyCurrentLimit, limitConfig.StatorCurrentLimit);
        torqueCurrentConfig.PeakReverseTorqueCurrent = -torqueCurrentConfig.PeakForwardTorqueCurrent;
    }

    public BreakerSwerveMotorCurrentLimitConfig(double limit) {
        limitsConfig = new CurrentLimitsConfigs();
        limitsConfig.StatorCurrentLimit = limit;
        limitsConfig.StatorCurrentLimitEnable = true;
        torqueCurrentConfig = new TorqueCurrentConfigs();
        torqueCurrentConfig.PeakForwardTorqueCurrent = limit;
        torqueCurrentConfig.PeakReverseTorqueCurrent = -limit;
    }

    public CurrentLimitsConfigs getLimitsConfig() {
        return limitsConfig;
    }

    public TorqueCurrentConfigs getTorqueCurrentConfig() {
        return torqueCurrentConfig;
    }
}
