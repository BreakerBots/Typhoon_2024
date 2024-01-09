// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.rev;

import java.util.Objects;
import java.util.Optional;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveSparkDutyCycleEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerSwerveAzimuthControler;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;
import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

/** Add your docs here. */
public class BreakerBrushlessSparkMaxSwerveModuleAngleMotor extends BreakerGenericSwerveModuleAngleMotor {
    private BreakerSwerveModuleAngleMotorConfig config;
    private CANSparkMax motor;
    private BreakerSwerveAzimuthEncoder encoder;
    private Rotation2d targetAngle;
    private BreakerSwerveAzimuthControler azimuthControler;
    public BreakerBrushlessSparkMaxSwerveModuleAngleMotor(CANSparkMax motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetDegrees, boolean isMotorInverted, BreakerSwerveModuleAngleMotorConfig config) {
        this.motor = motor;
        this.encoder = encoder;
        this.config = config;
        motor.restoreFactoryDefaults();
        deviceName = "NEO_Swerve_Angle_Motor_(" + motor.getDeviceId() + ")";
        motor.setInverted(isMotorInverted);
        encoder.config(false, encoderAbsoluteAngleOffsetDegrees);
        azimuthControler = null;
        if (encoder.getBaseEncoderType() == BreakerSwerveSparkDutyCycleEncoder.class) {
            SparkMaxPIDController sparkPID = motor.getPIDController();
            BreakerREVUtil.checkError(sparkPID.setP(config.getPIDConfig().kP), "Failed to config " + deviceName + " closed loop kP");
            BreakerREVUtil.checkError(sparkPID.setP(config.getPIDConfig().kI), "Failed to config " + deviceName + " closed loop kI");
            BreakerREVUtil.checkError(sparkPID.setP(config.getPIDConfig().kD), "Failed to config " + deviceName + " closed loop kD");
            BreakerREVUtil.checkError(sparkPID.setPositionPIDWrappingEnabled(true), "Failed to config " + deviceName + " closed loop wrapping enabled");
            BreakerREVUtil.checkError(sparkPID.setPositionPIDWrappingMaxInput(0.5), "Failed to config " + deviceName + " closed loop wrapping max input");
            BreakerREVUtil.checkError(sparkPID.setPositionPIDWrappingMinInput(-0.5), "Failed to config " + deviceName + " closed loop wrapping min input");
            BreakerREVUtil.checkError(sparkPID.setFeedbackDevice((AbsoluteEncoder) encoder.getBaseEncoder()), "Failed to config " + deviceName + " closed loop feedback device");
            azimuthControler = new BreakerSwerveAzimuthControler((Rotation2d tgt) -> {sparkPID.setReference(tgt.getRotations(), ControlType.kPosition);});
        }

        if (Objects.isNull(azimuthControler)) {
            azimuthControler = new BreakerSwerveAzimuthControler(motor::set, encoder, config.getPIDConfig());
        }

        Optional<Double> outputRampPeriod = config.getOutputRampPeriod();
        if (outputRampPeriod.isPresent()) {
            BreakerREVUtil.checkError(motor.setClosedLoopRampRate(outputRampPeriod.get()), "Failed to config " + deviceName + " closed loop ramp rate");
            BreakerREVUtil.checkError(motor.setOpenLoopRampRate(outputRampPeriod.get()), "Failed to config " + deviceName + " open loop ramp rate");
        }

        BreakerREVUtil.checkError(motor.enableVoltageCompensation(12.0), "Failed to config " + deviceName + " voltage compensation");
        BreakerREVUtil.checkError(motor.setSmartCurrentLimit((int)config.getSupplyCurrentLimit()),  "Failed to config " + deviceName + " smart current limit");
        targetAngle = new Rotation2d();
        motor.burnFlash();
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        azimuthControler.setTargetAngle(targetAngle);
        this.targetAngle = targetAngle;
        
    }

    @Override
    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(encoder.getAbsolute());
    }

    @Override
    public Rotation2d getRelativeAngle() {
        return Rotation2d.fromRotations(encoder.getRelative());
    }
    @Override
    public void setBrakeMode(boolean isEnabled) {
        motor.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        Pair<DeviceHealth, String> motorPair = BreakerREVUtil.getSparkMaxHealthAndFaults(motor.getFaults());
        Pair<DeviceHealth, String> encoderPair = encoder.getFaultData();
        if (encoderPair.getFirst() != DeviceHealth.NOMINAL || encoderPair.getFirst() != DeviceHealth.INOPERABLE) {
            health = DeviceHealth.INOPERABLE;
            if (motorPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ANGLE_MOTOR_FAULTS : " + motorPair.getSecond();
            }
            if (encoderPair.getFirst() != DeviceHealth.NOMINAL) {
                faultStr += " ENCODER_FAULTS : " + encoderPair.getSecond();
            }
        }
    }

    @Override
    public double getSupplyCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public double getMotorOutput() {
        return motor.getAppliedOutput();
    }

    @Override
    public BreakerSwerveModuleAngleMotorConfig getConfig() {
        return config;
    }

}
