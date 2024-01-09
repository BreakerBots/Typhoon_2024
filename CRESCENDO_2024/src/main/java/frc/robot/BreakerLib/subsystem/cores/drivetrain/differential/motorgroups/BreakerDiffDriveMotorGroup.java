// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.BreakerLib.devices.encoders.BreakerGenericEncoder;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;

/** Add your docs here. */
public abstract class BreakerDiffDriveMotorGroup extends BreakerSelfTestableBase {
    private MotorControllerGroup motorGroup;
    private DoubleSupplier encoderPositionRotationsSupplier, encoderVelocityRotationsPerSecondSupplier;

    protected BreakerDiffDriveMotorGroup(boolean invert, DoubleSupplier encoderPositionRotationsSupplier, DoubleSupplier encoderVelocityRotationsPerSecondSupplier, MotorController... motors) {
        motorGroup = new MotorControllerGroup(motors);
        motorGroup.setInverted(invert);
        this.encoderPositionRotationsSupplier = encoderPositionRotationsSupplier;
        this.encoderVelocityRotationsPerSecondSupplier = encoderVelocityRotationsPerSecondSupplier;
    }

    public abstract void setBrakeMode(boolean isEnabled);

    public abstract void setEncoderPosition(double newPosition);

    public void resetEncoderPosition() {
        setEncoderPosition(0.0);
    }

    public void set(double dutyCycle) {
        motorGroup.set(dutyCycle);
    }

    public void setVoltage(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    public double getOutputDutyCycle() {
        return motorGroup.get();
    }

    public double getOutputVoltage() {
        return motorGroup.get() * RobotController.getBatteryVoltage();
    }
  
    public double getEncoderPosition() {
        return (isInverted() ? -1 : 1) * encoderPositionRotationsSupplier.getAsDouble();
    }

    public double getEncoderVelocity() {
        return (isInverted() ? -1 : 1) * encoderVelocityRotationsPerSecondSupplier.getAsDouble();
    }

    public boolean isInverted() {
        return motorGroup.getInverted();
    }
}
