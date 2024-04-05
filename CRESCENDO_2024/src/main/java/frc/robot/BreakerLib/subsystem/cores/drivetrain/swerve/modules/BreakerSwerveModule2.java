// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.BreakerLib.devices.CANDeviceID;

/** Add your docs here. */
public class BreakerSwerveModule2 {
    public BreakerSwerveModule2(CANDeviceID driveMotorID, CANDeviceID azimuthMotorID, CANDeviceID azimuthEncoderID,  SwerveMotorConfig driveConfig, SwerveMotorConfig azimuthConfig)  {
       
    }



    public static record SwerveMotorKinimaticLimits (Measure<Velocity<Distance>> maxVel, Measure<Velocity<Velocity<Distance>>> maxAccel) {};
    public static record SwerveMotorControlGainConfig(double kP, double kI, double kD, double kA, double kS, double kV, SwerveMotorControlOutputUnits controlOutputUnits) {}
    public static class SwerveMotorCurrentLimits {
        private CurrentLimitsConfigs limitsConfig;
        private TorqueCurrentConfigs torqueCurrentConfig;
        public SwerveMotorCurrentLimits(CurrentLimitsConfigs limitConfig) {
            this.limitsConfig = limitConfig;
            torqueCurrentConfig = new TorqueCurrentConfigs();
            torqueCurrentConfig.PeakForwardTorqueCurrent = limitConfig.SupplyCurrentLimit;
            torqueCurrentConfig.PeakReverseTorqueCurrent = -limitConfig.SupplyCurrentLimit;
        }

        public SwerveMotorCurrentLimits(TorqueCurrentConfigs torqueCurrentConfig) {
            this.torqueCurrentConfig = torqueCurrentConfig;
            limitsConfig = new CurrentLimitsConfigs();
            limitsConfig.SupplyCurrentLimit = torqueCurrentConfig.PeakForwardTorqueCurrent;
            limitsConfig.SupplyCurrentThreshold = torqueCurrentConfig.PeakForwardTorqueCurrent;
            limitsConfig.StatorCurrentLimitEnable = true;
        }

        public SwerveMotorCurrentLimits(double limit) {
            limitsConfig = new CurrentLimitsConfigs();
            limitsConfig.SupplyCurrentLimit = limit;
            limitsConfig.SupplyCurrentThreshold = limit;
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
    public static record SwerveMotorConfig(SwerveMotorKinimaticLimits kinimaticLimits, SwerveMotorControlGainConfig controlGains, SwerveMotorCurrentLimits currentLimits, double gearRatio, boolean invert) {}
    
    
    public static enum SwerveMotorControlOutputUnits {
        TORQUE_CURRENT,
        VOLTAGE,
        DUTY_CYCLE
    }

    
}
