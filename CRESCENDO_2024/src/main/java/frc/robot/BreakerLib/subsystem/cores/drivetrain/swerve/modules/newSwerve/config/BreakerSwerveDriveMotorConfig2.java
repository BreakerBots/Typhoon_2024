// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.newSwerve.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Mult;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.BreakerLib.util.MechanismRatio;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/** Add your docs here. */
public class BreakerSwerveDriveMotorConfig2 {
    private final BreakerSwerveDriveMotorClosedLoopControlConfig closedLoopControlConfig;
    private final BreakerSwerveMotorCurrentLimitConfig currentLimitConfig;
    private final MechanismRatio rotorToWheelRatio;
    private final Measure<Distance> wheelRadius;
    private final Measure<Mult<Mass, Mult<Distance, Distance>>> intertia;
    private final Measure<Velocity<Distance>> maxVelocity;
    private final Measure<Velocity<Velocity<Distance>>> maxAcceleration;
    private final boolean invert;
    public BreakerSwerveDriveMotorConfig2(BreakerSwerveDriveMotorClosedLoopControlConfig closedLoopControlConfig, BreakerSwerveMotorCurrentLimitConfig currentLimitConfig, MechanismRatio rotorToWheelRatio, Measure<Distance> wheelRadius, Measure<Mult<Mass, Mult<Distance, Distance>>> intertia, Measure<Velocity<Distance>> maxVelocity, Measure<Velocity<Velocity<Distance>>> maxAcceleration, boolean invert) {
        this.closedLoopControlConfig = closedLoopControlConfig;
        this.currentLimitConfig = currentLimitConfig;
        this.rotorToWheelRatio = rotorToWheelRatio;
        this.wheelRadius = wheelRadius;
        this.intertia = intertia;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.invert = invert;
    }

    public BreakerSwerveDriveMotorConfig2() {
        this(new BreakerSwerveDriveMotorClosedLoopControlConfig(), new BreakerSwerveMotorCurrentLimitConfig(80.0), new MechanismRatio(), Units.Meters.of(0.0), BreakerUnits.KilogramSquareMetre.of(0.0), Units.MetersPerSecond.of(0.0), Units.MetersPerSecondPerSecond.of(0.0), false);
    }

    public BreakerSwerveDriveMotorConfig2 withClosedLoopControlConfig(BreakerSwerveDriveMotorClosedLoopControlConfig closedLoopControlConfig) {
        return new BreakerSwerveDriveMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToWheelRatio, wheelRadius, intertia, maxVelocity, maxAcceleration, invert);
    }

    public BreakerSwerveDriveMotorConfig2 withCurrentLimitConfig(BreakerSwerveMotorCurrentLimitConfig currentLimitConfig) {
        return new BreakerSwerveDriveMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToWheelRatio, wheelRadius, intertia, maxVelocity, maxAcceleration, invert);
    }

    public BreakerSwerveDriveMotorConfig2 withRotorToWheelRatio(MechanismRatio rotorToWheelRatio) {
        return new BreakerSwerveDriveMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToWheelRatio, wheelRadius, intertia, maxVelocity, maxAcceleration, invert);
    }

    public BreakerSwerveDriveMotorConfig2 withWheelRadius(Measure<Distance> wheelRadius) {
        return new BreakerSwerveDriveMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToWheelRatio, wheelRadius, intertia, maxVelocity, maxAcceleration, invert);
    }

    public BreakerSwerveDriveMotorConfig2 withIntertia(Measure<Mult<Mass, Mult<Distance, Distance>>> intertia) {
        return new BreakerSwerveDriveMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToWheelRatio, wheelRadius, intertia, maxVelocity, maxAcceleration, invert);
    }

    public BreakerSwerveDriveMotorConfig2 withMaxVelocity(Measure<Velocity<Distance>> maxVelocity) {
        return new BreakerSwerveDriveMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToWheelRatio, wheelRadius, intertia, maxVelocity, maxAcceleration, invert);
    }

    public BreakerSwerveDriveMotorConfig2 withMaxAcceleration(Measure<Velocity<Velocity<Distance>>> maxAcceleration) {
        return new BreakerSwerveDriveMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToWheelRatio, wheelRadius, intertia, maxVelocity, maxAcceleration, invert);
    }

    public BreakerSwerveDriveMotorConfig2 withInvert(boolean invert) {
        return new BreakerSwerveDriveMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToWheelRatio, wheelRadius, intertia, maxVelocity, maxAcceleration, invert);
    }

    public BreakerSwerveDriveMotorClosedLoopControlConfig getClosedLoopControlConfig() {
        return closedLoopControlConfig;
    }

    public BreakerSwerveMotorCurrentLimitConfig getCurrentLimitConfig() {
        return currentLimitConfig;
    }

    public Measure<Velocity<Velocity<Distance>>> getMaxAcceleration() {
        return maxAcceleration;
    }

    public Measure<Velocity<Distance>> getMaxVelocity() {
        return maxVelocity;
    }

    public MechanismRatio getRotorToWheelRatio() {
        return rotorToWheelRatio;
    }

    public Measure<Distance> getWheelRadius() {
        return wheelRadius;
    }

    public Measure<Mult<Mass, Mult<Distance, Distance>>> getIntertia() {
        return intertia;
    }

    public boolean getInvert() {
        return invert;
    }
 
    public static class BreakerSwerveDriveMotorClosedLoopControlConfig {
        private final double kP, kI, kD, kV, kA, kS;
        private final BreakerSwerveDriveMotorClosedLoopControlType controlType;
        public BreakerSwerveDriveMotorClosedLoopControlConfig(double kP, double kI, double kD, double kV, double kA, double kS, BreakerSwerveDriveMotorClosedLoopControlType controlType) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kA = kA;
            this.kS = kS;
            this.controlType = controlType;
        }

        public BreakerSwerveDriveMotorClosedLoopControlConfig() {
            this(0.0, 0.0, 0.0, 0.0 ,0.0, 0.0, BreakerSwerveDriveMotorClosedLoopControlType.VOLTAGE);
        }

        public BreakerSwerveDriveMotorClosedLoopControlConfig withControlType(BreakerSwerveDriveMotorClosedLoopControlType controlType) {
            return new BreakerSwerveDriveMotorClosedLoopControlConfig(getP(), getI(), getD(), getV(), getA(), getS(), controlType);
        }

        public BreakerSwerveDriveMotorClosedLoopControlType getControlType() {
            return controlType;
        }

        public static enum BreakerSwerveDriveMotorClosedLoopControlType {
            TORQUE_CURRENT,
            VOLTAGE,
            DUTY_CYCLE
        }

        public BreakerSwerveDriveMotorClosedLoopControlConfig withP(double kP) {
            return new BreakerSwerveDriveMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, controlType);
        }

        public BreakerSwerveDriveMotorClosedLoopControlConfig withI(double kI) {
            return new BreakerSwerveDriveMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, controlType);
        }
        public BreakerSwerveDriveMotorClosedLoopControlConfig withD(double kD) {
            return new BreakerSwerveDriveMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, controlType);
        }
        public BreakerSwerveDriveMotorClosedLoopControlConfig withV(double kV) {
            return new BreakerSwerveDriveMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, controlType);
        }
        public BreakerSwerveDriveMotorClosedLoopControlConfig withA(double kA) {
            return new BreakerSwerveDriveMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, controlType);
        }
        public BreakerSwerveDriveMotorClosedLoopControlConfig withS(double kS) {
            return new BreakerSwerveDriveMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, controlType);
        }

        public double getA() {
            return kA;
        }

        public double getD() {
            return kD;
        }

        public double getI() {
            return kI;
        }

        public double getP() {
            return kP;
        }

        public double getS() {
            return kS;
        }

        public double getV() {
            return kV;
        }
    }
}
