// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import javax.security.auth.PrivateCredentialPermission;

import org.opencv.ml.StatModel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import edu.wpi.first.units.Per;
import frc.robot.BreakerLib.devices.CANDeviceID;
// import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util.LatencyCompensatedMeasuredStatusSignal;
// import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util.MeasuredStatusSignal;

/** Add your docs here. */
public class BreakerSwerveModule2 {
    private TalonFX driveMotor, azimuthMotor;
    private CANcoder azimuthEncoder;
    private SwerveModuleConfig config;
    private StatusSignal<Double> drivePositionSignal, driveVelocitySignal;
    private StatusSignal<Double> azimuthPositionSignal, azimuthVelocitySignal;
    private BaseStatusSignal[] signals = new BaseStatusSignal[]{drivePositionSignal, driveVelocitySignal, azimuthPositionSignal, azimuthVelocitySignal};

    public SwerveModuleState targetState;

    private final VelocityDutyCycle velocityDutyCycleRequest;
    private final VelocityVoltage velocityVoltageRequest;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest;

    private final MotionMagicDutyCycle motionMagicDutyCycleRequest;
    private final MotionMagicVoltage motionMagicVoltageRequest;
    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentRequest;

    public BreakerSwerveModule2(CANDeviceID driveMotorID, CANDeviceID azimuthMotorID, CANDeviceID azimuthEncoderID, SwerveModuleConfig config)  {
        this.config = config;
       driveMotor = new TalonFX(driveMotorID.getDeviceID(), driveMotorID.getBusName());
       azimuthMotor = new TalonFX(azimuthMotorID.getDeviceID(), azimuthMotorID.getBusName());
       azimuthEncoder = new CANcoder(azimuthEncoderID.getDeviceID(), azimuthEncoderID.getBusName());
        

    }

    private void createControlRequests() {
        motionMagicDutyCycleRequest = new 
    }

    private void configDriveMotor() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            driveConfig.Feedback.SensorToMechanismRatio = config.getDriveMotorConfig().gearRatio();
            driveConfig.Slot0.kP = config.getDriveMotorConfig().controlGains().kP;
            driveConfig.Slot0.kI = config.getDriveMotorConfig().controlGains().kI;
            driveConfig.Slot0.kD = config.getDriveMotorConfig().controlGains().kD;
            driveConfig.Slot0.kA = config.getDriveMotorConfig().controlGains().kA;
            driveConfig.Slot0.kS = config.getDriveMotorConfig().controlGains().kS;
            driveConfig.Slot0.kV = config.getDriveMotorConfig().controlGains().kV;
            driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveConfig.CurrentLimits = config.getDriveMotorConfig().currentLimits().getLimitsConfig();
            driveConfig.TorqueCurrent = config.getDriveMotorConfig().currentLimits().getTorqueCurrentConfig();
            
    }

    public SwerveModulePosition getPosition(boolean refresh, boolean compensateForLatency) {
        if (refresh) {
            refreshAllSignals();
        }
        double drivePos = drivePositionSignal.getValueAsDouble();
        double azimuthPos = azimuthPositionSignal.getValueAsDouble();
        if (compensateForLatency) {
            drivePos =  BaseStatusSignal.getLatencyCompensatedValue(drivePositionSignal, driveVelocitySignal);
            azimuthPos = BaseStatusSignal.getLatencyCompensatedValue(azimuthPositionSignal, azimuthVelocitySignal);
        }
        return new SwerveModulePosition(drivePos / config.getDriveRotorRotationsPerMeter(), Rotation2d.fromRotations(azimuthPos));
    } 

    public SwerveModuleState getState(boolean refresh, boolean compensateForLatency) {
        if (refresh) {
            refreshAllSignals();
        }
        double driveVel = driveVelocitySignal.getValueAsDouble();
        double azimuthPos = azimuthPositionSignal.getValueAsDouble();
        if (compensateForLatency) {
            azimuthPos = BaseStatusSignal.getLatencyCompensatedValue(azimuthPositionSignal, azimuthVelocitySignal);
        }
        return new SwerveModuleState(driveVel / config.getDriveRotorRotationsPerMeter(), Rotation2d.fromDegrees(azimuthPos));
    }

    public BaseStatusSignal[] getSignals() {
        return signals;
    }

    public void refreshAllSignals() {
        BaseStatusSignal.refreshAll(signals);
    }

    public void setTargetState(SwerveModuleState state, boolean isDriveOpenLoop, boolean optimize, boolean refreshRefrencePosition) {
        Rotation2d curAzimuthAng = getPosition(refreshRefrencePosition, true).angle;
        targetState = state;
        if (optimize) {
            targetState = SwerveModuleState.optimize(targetState, curAzimuthAng); 
        }
        double angleToSet = targetState.angle.getRotations();

        switch (config.getAzimuthMotorConfig().controlGains().controlOutputUnits()) {
            case DUTY_CYCLE:
                break;
            case TORQUE_CURRENT:
                break;
            case VOLTAGE:
            default:
                break;
            
        }



        double velocityToSet = targetState.speedMetersPerSecond * config.getDriveRotorRotationsPerMeter();
        double steerMotorError = angleToSet - curAzimuthAng.getRotations();
        double cosineScalar = Math.cos(steerMotorError * 2.0 * Math.PI);
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }
        velocityToSet *= cosineScalar;
        double azimuthTurnRps = azimuthVelocitySignal.getValue();
        double driveRateBackOut = azimuthTurnRps * config.getCouplingGearRatio();
        velocityToSet += driveRateBackOut;



    }


    public static record SwerveModuleDriveKinimaticLimits(Measure<Velocity<Distance>> maxLinearVelocity, Measure<Velocity<Velocity<Distance>>> maxLinearAcceleration) {};
    public static record SwerveModuleAzimuthMotionProfileConstraints(Measure<Velocity<Angle>> goalAngularVelocity, Measure<Velocity<Velocity<Distance>>> goalAngularAcceleration, double expoKv, double expoKa) {};
    public static class SwerveModuleAzimuthMotionProfile {

    }
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

        public SwerveMotorCurrentLimits(double limit) {
            limitsConfig = new CurrentLimitsConfigs();
            limitsConfig.StatorCurrentLimit = limit;
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
        private SwerveMotorConfig driveMotorConfig, azimuthMotorConfig;
        private Rotation2d azimuthEncoderOffset;
        private Measure<Distance> wheelRadius;
        private double couplingGearRatio;
        private Translation2d modulePosition;

        public Rotation2d getAzimuthEncoderOffset() {
            return azimuthEncoderOffset;
        }

        public SwerveMotorConfig getAzimuthMotorConfig() {
            return azimuthMotorConfig;
        }

        public double getCouplingGearRatio() {
            return couplingGearRatio;
        }

        public SwerveMotorConfig getDriveMotorConfig() {
            return driveMotorConfig;
        }
        
        public Translation2d getModulePosition() {
            return modulePosition;
        }

        public Measure<Distance> getWheelRadius() {
            return wheelRadius;
        }

        public double getDriveRotorRotationsPerMeter() {
            return driveMotorConfig.gearRatio() / (2.0 * Math.PI * wheelRadius.in(Units.Meters));
        }
    } 

    public static enum SwerveMotorControlOutputUnits {
        TORQUE_CURRENT,
        VOLTAGE,
        DUTY_CYCLE
    }

    

    
}
