// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;

/** Add your docs here. */
public class BreakerSwerveAzimuthControler {
    private PIDController pid;
    private Consumer<Double> rawMotorOutputConsumer;
    private BreakerSwerveAzimuthEncoder encoder;
    private Consumer<Rotation2d> targetAngleConsumer;
    public BreakerSwerveAzimuthControler(Consumer<Double> rawMotorOutputConsumer, BreakerSwerveAzimuthEncoder encoder, BreakerSwerveMotorPIDConfig pidConfig) {
        this.encoder = encoder;
        this.rawMotorOutputConsumer = rawMotorOutputConsumer;
        pid = new PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD);
        pid.enableContinuousInput(-0.5, 0.5);
    }

    public BreakerSwerveAzimuthControler(Consumer<Rotation2d> targetAngleConsumer) {
        this.targetAngleConsumer = targetAngleConsumer;
    }

    public void setTargetAngle(Rotation2d target) {
        if (Objects.isNull(targetAngleConsumer)) {
            rawMotorOutputConsumer.accept(pid.calculate(encoder.getAbsolute(), target.getRotations()));
            return;
        }
        targetAngleConsumer.accept(target);
    }
} 
