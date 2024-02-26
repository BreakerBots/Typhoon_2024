// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

/** Add your docs here. */
public class BreakerProTalonFXSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
    private TalonFX motor;
    private BreakerSwerveModuleDriveMotorConfig config;
    private double driveGearRatio, wheelDiameter, targetVelocity, maxAttainableWheelSpeed;
    private final DutyCycleOut openLoopDutyCycleRequest;
    private final VelocityDutyCycle velocityDutyCycleRequest;
    private final VelocityVoltage velocityVoltageRequest;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest;
    private final double wheelCircumfrenceMeters;
    private BreakerArbitraryFeedforwardProvider arbFF;
    private final  ProTalonFXControlOutputUnits controlOutputUnits;
    public BreakerProTalonFXSwerveModuleDriveMotor(TalonFX motor, boolean isMotorInverted, ProTalonFXControlOutputUnits controlOutputUnits, BreakerSwerveModuleDriveMotorConfig config) {
        this.motor = motor;
        this.config = config;
        this.driveGearRatio = config.getDriveGearRatio();
        this.wheelDiameter = config.getWheelDiameter();
        this.arbFF = config.getArbFF();
        this.maxAttainableWheelSpeed = config.getMaxAttainableWheelSpeed();
        wheelCircumfrenceMeters = wheelDiameter*Math.PI;
        targetVelocity = 0.0;
        velocityDutyCycleRequest = new VelocityDutyCycle(0.0, 0.0, true, 0.0, 1, false, false, false);
        velocityVoltageRequest = new VelocityVoltage(0.0, 0.0, true, 0.0, 1, false, false, false);
        velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0, 0.0, 1, 1, false, false, false);
        openLoopDutyCycleRequest = new DutyCycleOut(0.0, true, false, false, false);
        this.controlOutputUnits = controlOutputUnits;
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = driveGearRatio;
        driveConfig.Slot1.kP = config.getPIDConfig().kP;
        driveConfig.Slot1.kI = config.getPIDConfig().kI;
        driveConfig.Slot1.kD = config.getPIDConfig().kD;
        driveConfig.Slot1.kV = config.getPIDConfig().kF;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        if (controlOutputUnits != ProTalonFXControlOutputUnits.TORQUE_CURRENT) {
            driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
            driveConfig.CurrentLimits.SupplyCurrentThreshold = 40;
            driveConfig.CurrentLimits.SupplyTimeThreshold = 0.05;
            driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        } else {
            driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.getSupplyCurrentLimit();
            driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -config.getSupplyCurrentLimit();
        }
        Optional<Double> outputRampPeriod = config.getOutputRampPeriod();
        if (outputRampPeriod.isPresent()) {
            driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = outputRampPeriod.get();
            driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = outputRampPeriod.get();
            driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = outputRampPeriod.get();
            driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = outputRampPeriod.get();
        }
        BreakerPhoenix6Util.checkStatusCode(motor.getConfigurator().apply(driveConfig),
                " Failed to config swerve module drive motor ");
    
        motor.setInverted(isMotorInverted);
        setMotorVel(0.0, 0.0);
        BreakerPhoenix6Util.setBrakeMode(motor, isMotorInverted);
    }

    private void setMotorVel(double vel, double ff) {
        switch (controlOutputUnits) {
            case DUTY_CYCLE:
                motor.setControl(velocityDutyCycleRequest.withVelocity(vel).withFeedForward(ff));
                break;
            case TORQUE_CURRENT:
                motor.setControl(velocityTorqueCurrentRequest.withVelocity(vel).withFeedForward(ff)/*.withOverrideCoastDurNeutral(MathUtil.isNear(0.0, vel, 1E-9))*/);
                break;
            case VOLTAGE:
            default:
                motor.setControl(velocityVoltageRequest.withVelocity(vel).withFeedForward(ff));
                break;
        }
    }

    private void setControlOpenLoop(double vel) {
        motor.setControl(openLoopDutyCycleRequest.withOutput(vel/maxAttainableWheelSpeed));
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        Pair<DeviceHealth, String> pair = BreakerPhoenix6Util.checkMotorFaultsAndConnection(motor);
        health = pair.getFirst();
        if (health != DeviceHealth.NOMINAL) {
            faultStr = " DRIVE_MOTOR_FAULTS : " + pair.getSecond();
        }
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond, boolean isOpenLoop) {
        targetVelocity = targetMetersPerSecond;
        if (isOpenLoop) {
            setControlOpenLoop(targetMetersPerSecond);
        } else {
            setMotorVel(targetMetersPerSecond / wheelCircumfrenceMeters, arbFF.getArbitraryFeedforwardValue(targetMetersPerSecond));
        }
    }

    @Override
    public double getVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(motor.getVelocity(), motor.getAcceleration()) * wheelCircumfrenceMeters;
    }

    @Override
    public double getDistance() {
        return BaseStatusSignal.getLatencyCompensatedValue(motor.getPosition(), motor.getVelocity()) * wheelCircumfrenceMeters;
    }

    @Override
    public void resetDistance() {
        BreakerPhoenix6Util.checkStatusCode(motor.setPosition(0.0),
                " Failed to reset swerve module rive motor position ");
        ;
    }

    @Override
    public void setBrakeMode(boolean isEnabled) {
        BreakerPhoenix6Util.setBrakeMode(motor, isEnabled);
    }

    @Override
    public double getTargetVelocity() {
        return targetVelocity;
    }

    @Override
    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValue();
    }

    @Override
    public double getMotorOutput() {
        return motor.getClosedLoopOutput().getValue();
    }

    public enum ProTalonFXControlOutputUnits {
        /** More accurate, compensates for supply voltage veriance, generaly less max power avalible compaired to duty cycle control. PID/FF should return outputs in Volts */
        VOLTAGE,
        /** Less accurate, does not compensate for supply voltage veriance, generaly more power avalible compired to voltage control. PID/FF should return outputs in percent out [-1.0, 1.0] */
        DUTY_CYCLE,
        /** Balenced, most powerful, very accurate. PID/FF should return outputs in Ampiers */
        TORQUE_CURRENT
    }

    @Override
    public BreakerSwerveModuleDriveMotorConfig getConfig() {
        return config;
    }

    @Override
    public void setStatusUpdatePeriod(double period) {
        BaseStatusSignal.setUpdateFrequencyForAll(1.0/period, motor.getPosition(), motor.getVelocity(), motor.getAcceleration());
    }
}
