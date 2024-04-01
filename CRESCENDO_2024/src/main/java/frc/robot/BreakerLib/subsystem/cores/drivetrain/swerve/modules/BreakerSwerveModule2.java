// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;

/** Add your docs here. */
public class BreakerSwerveModule2 {
    public BreakerSwerveModule2(TalonFX driveMotor, TalonFX azimuth, CANcoder azimuthEncoder, SwerveMotorConfig driveConfig, SwerveMotorConfig azimuthConfig)  {

    }

    public static record SwerveMotorKinimaticLimits (Measure<Velocity<Distance>> maxVel, Measure<Velocity<Velocity<Distance>>> maxAccel) {};
    public static record SwerveMotorControlGainConfig(double kP, double kI, double kD, double kA, double kS, double kV, SwerveMotorControlOutputUnits controlOutputUnits) {}
    public static record SwerveMotorCurrentLimits(Measure<Current> limit, Measure<Current> threashold, Measure<Time> thresholdTime, Measure<Current> slipCurrent) {}
    public static record SwerveMotorConfig(SwerveMotorKinimaticLimits kinimaticLimits, SwerveMotorControlGainConfig controlGains, SwerveMotorCurrentLimits currentLimits, double gearRatio, boolean invert) {
    }
    
    
    public static enum SwerveMotorControlOutputUnits {
        TORQUE_CURRENT,
        VOLTAGE,
        DUTY_CYCLE
    }

    
}
