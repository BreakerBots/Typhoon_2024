// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.newSwerve.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Mult;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.BreakerLib.util.MechanismRatio;

/** Add your docs here. */
public class BreakerSwerveAzimuthMotorConfig2 {
    private final BreakerSwerveAzimuthMotorClosedLoopControlConfig closedLoopControlConfig;
    private final BreakerSwerveMotorCurrentLimitConfig currentLimitConfig;
    private final MechanismRatio rotorToEncoderRatio, encoderToAzimuthRatio;
    private final Measure<Mult<Mass, Mult<Distance, Distance>>> intertia;
    private final Measure<Velocity<Angle>> maxVelocity;
    private final boolean invert;
    public BreakerSwerveAzimuthMotorConfig2(BreakerSwerveAzimuthMotorClosedLoopControlConfig closedLoopControlConfig, BreakerSwerveMotorCurrentLimitConfig currentLimitConfig, MechanismRatio rotorToEncoderRatio, MechanismRatio encoderToAzimuthRatio, Measure<Mult<Mass, Mult<Distance, Distance>>> intertia, Measure<Velocity<Angle>> maxVelocity, boolean invert) {
        this.closedLoopControlConfig = closedLoopControlConfig;
        this.currentLimitConfig = currentLimitConfig;
        this.rotorToEncoderRatio = rotorToEncoderRatio;
        this.encoderToAzimuthRatio = encoderToAzimuthRatio;
        this.intertia = intertia;
        this.maxVelocity = maxVelocity;
        this.invert = invert;
    }

    public BreakerSwerveAzimuthMotorConfig2 withClosedLoopControlConfig(BreakerSwerveAzimuthMotorClosedLoopControlConfig closedLoopControlConfig) {
        return new BreakerSwerveAzimuthMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToEncoderRatio, encoderToAzimuthRatio, intertia, maxVelocity, invert);
    }

    public BreakerSwerveAzimuthMotorConfig2 withCurrentLimitConfig(BreakerSwerveMotorCurrentLimitConfig currentLimitConfig) {
        return new BreakerSwerveAzimuthMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToEncoderRatio, encoderToAzimuthRatio, intertia, maxVelocity, invert);
    }

    public BreakerSwerveAzimuthMotorConfig2 withRotorToEncoderRatio(MechanismRatio rotorToEncoderRatio) {
        return new BreakerSwerveAzimuthMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToEncoderRatio, encoderToAzimuthRatio, intertia, maxVelocity, invert);
    }

    public BreakerSwerveAzimuthMotorConfig2 withEncoderToAzimuthRatio(MechanismRatio encoderToAzimuthRatio) {
        return new BreakerSwerveAzimuthMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToEncoderRatio, encoderToAzimuthRatio, intertia, maxVelocity, invert);
    }

    public BreakerSwerveAzimuthMotorConfig2 withIntertia(Measure<Mult<Mass, Mult<Distance, Distance>>> intertia) {
        return new BreakerSwerveAzimuthMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToEncoderRatio, encoderToAzimuthRatio, intertia, maxVelocity, invert);
    }

    public BreakerSwerveAzimuthMotorConfig2 withMaxVelocity(Measure<Velocity<Angle>> maxVelocity) {
        return new BreakerSwerveAzimuthMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToEncoderRatio, encoderToAzimuthRatio, intertia, maxVelocity, invert);
    }

    public BreakerSwerveAzimuthMotorConfig2 withInvert(boolean invert) {
        return new BreakerSwerveAzimuthMotorConfig2(closedLoopControlConfig, currentLimitConfig, rotorToEncoderRatio, encoderToAzimuthRatio, intertia, maxVelocity, invert);
    }

    public BreakerSwerveMotorCurrentLimitConfig getCurrentLimitConfig() {
        return currentLimitConfig;
    }

    public BreakerSwerveAzimuthMotorClosedLoopControlConfig getClosedLoopControlConfig() {
        return closedLoopControlConfig;
    }

    public MechanismRatio getEncoderToAzimuthRatio() {
        return encoderToAzimuthRatio;
    }

    public Measure<Mult<Mass, Mult<Distance, Distance>>> getIntertia() {
        return intertia;
    }

    public Measure<Velocity<Angle>> getMaxVelocity() {
        return maxVelocity;
    }

    public MechanismRatio getRotorToEncoderRatio() {
        return rotorToEncoderRatio;
    }

    public boolean getInvert() {
        return invert;
    }
 
    public static class BreakerSwerveAzimuthMotorClosedLoopControlConfig {
        private final double kP, kI, kD, kV, kA, kS, kExpoV, kExpoA;
        private final Measure<Velocity<Angle>> motionMagicCruiseVelocity;
        private final Measure<Velocity<Velocity<Angle>>> motionMagicAcceleration;
        private final Measure<Velocity<Velocity<Velocity<Angle>>>> motionMagicJerk;
        private final BreakerSwerveAzimuthMotorClosedLoopControlType controlType;
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig(double kP, double kI, double kD, double kV, double kA, double kS, double kExpoV, double kExpoA, Measure<Velocity<Angle>> motionMagicCruiseVelocity, Measure<Velocity<Velocity<Angle>>> motionMagicAcceleration, Measure<Velocity<Velocity<Velocity<Angle>>>> motionMagicJerk, BreakerSwerveAzimuthMotorClosedLoopControlType controlType) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kA = kA;
            this.kS = kS;
            this.kExpoV = kExpoV;
            this.kExpoA = kExpoA;
            this.motionMagicCruiseVelocity = motionMagicCruiseVelocity;
            this.motionMagicAcceleration = motionMagicAcceleration;
            this.motionMagicJerk = motionMagicJerk;
            this.controlType = controlType;
        }

        public BreakerSwerveAzimuthMotorClosedLoopControlType getControlType() {
            return controlType;
        }

        public static enum BreakerSwerveAzimuthMotorClosedLoopControlType {
            TRAPIZODAL_TORQUE_CURRENT,
            TRAPIZODAL_VOLTAGE,
            TRAPIZODAL_DUTY_CYCLE,
            EXPONENTIAL_TORQUE_CURRENT,
            EXPONENTIAL_VOLTAGE,
            EXPONENTIAL_DUTY_CYCLE
        }

        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withControlType(BreakerSwerveAzimuthMotorClosedLoopControlType controlType) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(getP(), getI(), getD(), getV(), getA(), getS(), kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }

        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withP(double kP) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }

        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withI(double kI) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withD(double kD) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withV(double kV) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withA(double kA) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withS(double kS) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withExpoV(double kExpoV) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withExpoA(double kExpoA) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withMotionMagicCruiseVelocity(Measure<Velocity<Angle>> motionMagicCruiseVelocity) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withMotionMagicAcceleration(Measure<Velocity<Velocity<Angle>>> motionMagicAcceleration) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
        }
        public BreakerSwerveAzimuthMotorClosedLoopControlConfig withMotionMagicJerk(Measure<Velocity<Velocity<Velocity<Angle>>>> motionMagicJerk) {
            return new BreakerSwerveAzimuthMotorClosedLoopControlConfig(kP, kI, kD, kV, kA, kS, kExpoV, kExpoA, motionMagicCruiseVelocity, motionMagicAcceleration, motionMagicJerk, controlType);
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

        public double getExpoA() {
            return kExpoA;
        }
        
        public double getExpoV() {
            return kExpoV;
        }

        public Measure<Velocity<Angle>> getMotionMagicCruiseVelocity() {
            return motionMagicCruiseVelocity;
        }

        public Measure<Velocity<Velocity<Angle>>> getMotionMagicAcceleration() {
            return motionMagicAcceleration;
        }

        public Measure<Velocity<Velocity<Velocity<Angle>>>> getMotionMagicJerk() {
            return motionMagicJerk;
        }
    }
}
