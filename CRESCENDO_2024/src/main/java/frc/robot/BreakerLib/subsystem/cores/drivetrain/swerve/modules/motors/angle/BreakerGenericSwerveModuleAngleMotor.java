// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.BreakerGenericSwerveModuleMotor;
import frc.robot.BreakerLib.util.logging.advantagekit.LogTable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

/** Add your docs here. */
public abstract class BreakerGenericSwerveModuleAngleMotor extends BreakerGenericSwerveModuleMotor {
    public abstract void setTargetAngle(Rotation2d targetAngle);
    public abstract Rotation2d getAbsoluteAngle();
    public abstract Rotation2d getRelativeAngle(); 
    public abstract void setBrakeMode(boolean isEnabled);
    public abstract Rotation2d getTargetAngle();
    public abstract BreakerSwerveAzimuthEncoder getEncoder();
    public abstract BreakerSwerveModuleAngleMotorConfig getConfig();
    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        table.put("AbsoluteAngleDeg", getAbsoluteAngle().getDegrees());
        table.put("AbsoluteTargetAngleDeg", getTargetAngle().getDegrees());
        table.put("RelativeAngleDeg", getRelativeAngle().getDegrees());
        
    }

    public static interface BreakerSwerveModuleAngleMotorHardwareDependentInterface {
        public abstract Optional<StatusSignal<Double>> get
    }

    public static class BreakerSwerveModuleAngleMotorConfig {
        private double azimuthGearRatio;
        private double supplyCurrentLimit;
        private Optional<Double> outputRampPeriod;
        private BreakerSwerveMotorPIDConfig pidConfig;
        public BreakerSwerveModuleAngleMotorConfig(double azimuthGearRatio, double supplyCurrentLimit, double outputRampPeriod,  BreakerSwerveMotorPIDConfig pidConfig) {
            this.azimuthGearRatio = azimuthGearRatio;
            this.supplyCurrentLimit = supplyCurrentLimit;
            this.outputRampPeriod = Optional.of(outputRampPeriod);
            this.pidConfig = pidConfig;
        }

        public BreakerSwerveModuleAngleMotorConfig(double azimuthGearRatio, double supplyCurrentLimit, BreakerSwerveMotorPIDConfig pidConfig) {
            this.azimuthGearRatio = azimuthGearRatio;
            this.supplyCurrentLimit = supplyCurrentLimit;
            this.outputRampPeriod = Optional.empty();
            this.pidConfig = pidConfig;
        }

        public BreakerSwerveModuleAngleMotorConfig withOutputRampPeriod(double outputRampPeriod) {
            this.outputRampPeriod = Optional.of(outputRampPeriod);
            return this;
        }

        public double getAzimuthGearRatio() {
            return azimuthGearRatio;
        }

        public double getSupplyCurrentLimit() {
            return supplyCurrentLimit;
        }

        public Optional<Double> getOutputRampPeriod() {
            return outputRampPeriod;
        }

        public BreakerSwerveMotorPIDConfig getPIDConfig() {
            return pidConfig;
        }
    }

}
