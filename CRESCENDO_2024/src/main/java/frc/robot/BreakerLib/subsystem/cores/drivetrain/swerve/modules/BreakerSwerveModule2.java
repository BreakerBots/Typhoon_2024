// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import org.opencv.ml.StatModel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Mult;
import frc.robot.BreakerLib.devices.CANDeviceID;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util.LatencyCompensatedStatusSignal;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util.MeasuredStatusSignal;

/** Add your docs here. */
public class BreakerSwerveModule2 {
    private TalonFX driveMotor, azimuthMotor;

    private StatusSignal driveDistanceSignal, driveVelocitySignal, driveAccelerationSignal;
    public BreakerSwerveModule2(CANDeviceID driveMotorID, CANDeviceID azimuthMotorID, CANDeviceID azimuthEncoderID,  SwerveMotorConfig driveConfig, SwerveMotorConfig azimuthConfig)  {
       
    }

    public LatencyCompensatedStatusSignal<Distance> getDriveDistance() {
        driveDistanceSignal.refresh();
        driveVelocitySignal.refresh();
        return new LatencyCompensatedStatusSignal<Distance>(driveDistanceSignal, driveVelocitySignal, Units.Meters);
    }

    public LatencyCompensatedStatusSignal<Velocity<Distance>> getDriveVelocity() {
        driveVelocitySignal.refresh();
        driveAccelerationSignal.refresh();
        return new LatencyCompensatedStatusSignal<Velocity<Distance>>(driveVelocitySignal, driveAccelerationSignal, Units.MetersPerSecond);
    }

    public MeasuredStatusSignal<Velocity<Velocity<Distance>>> getDriveAcceleration() {
        driveAccelerationSignal.refresh();
        return new MeasuredStatusSignal<Velocity<Velocity<Distance>>>(driveAccelerationSignal, Units.MetersPerSecondPerSecond);
    }

    public static record SwerveModuleDriveKinimaticLimits(Measure<Velocity<Distance>> maxLinearVelocity, Measure<Velocity<Velocity<Distance>>> maxLinearAcceleration) {};
    public static record SwerveModuleAzimuthMotionProfileConstraints(Measure<Velocity<Angle>> goalAngularVelocity, Measure<Velocity<Velocity<Distance>>> goalAngularAcceleration) {};
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
    public static record SwerveMotorConfig(SwerveMotorControlGainConfig controlGains, SwerveMotorCurrentLimits currentLimits, double gearRatio, Measure<Mult<Mass, Mult<Distance, Distance>>> intertia, boolean invert) {}
    public static class SwerveModuleConfig {
        private SwerveMotorConfig driveConfig, azimuthConfig;
        private Rotation2d azimuthEncoderOffset;
        private Measure<Distance> wheelRadius;
        private double couplingGearRatio;
        private Translation2d modulePosition;
    }

    public static enum SwerveMotorControlOutputUnits {
        TORQUE_CURRENT,
        VOLTAGE,
        DUTY_CYCLE
    }

    
}
