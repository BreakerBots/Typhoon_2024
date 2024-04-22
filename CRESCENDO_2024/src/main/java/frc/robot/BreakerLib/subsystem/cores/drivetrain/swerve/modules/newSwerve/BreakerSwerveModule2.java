// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.newSwerve;

// import javax.security.auth.PrivateCredentialPermission;

// import org.opencv.ml.StatModel;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfigurator;
// import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
// import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
// import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.units.Angle;
// import edu.wpi.first.units.Current;
// import edu.wpi.first.units.Distance;
// import edu.wpi.first.units.Mass;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.Time;
// import edu.wpi.first.units.Unit;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.Velocity;
// import edu.wpi.first.units.Mult;
// import edu.wpi.first.units.Per;
// import frc.robot.BreakerLib.devices.CANDeviceID;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.newSwerve.config.BreakerSwerveModuleConfig;

// /** Add your docs here. */
// public class BreakerSwerveModule2 {
//     private TalonFX driveMotor, azimuthMotor;
//     private CANcoder azimuthEncoder;
//     private BreakerSwerveModuleConfig config;

//     private StatusSignal<Double> drivePositionSignal, driveVelocitySignal;
//     private StatusSignal<Double> azimuthPositionSignal, azimuthVelocitySignal;
//     private BaseStatusSignal[] signals = new BaseStatusSignal[]{drivePositionSignal, driveVelocitySignal, azimuthPositionSignal, azimuthVelocitySignal};

//     public SwerveModuleState targetState;

//     private final VelocityDutyCycle velocityDutyCycleRequest;
//     private final VelocityVoltage velocityVoltageRequest;
//     private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest;

//     private final MotionMagicDutyCycle motionMagicDutyCycleRequest;
//     private final MotionMagicVoltage motionMagicVoltageRequest;
//     private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentRequest;

//     private final MotionMagicExpoDutyCycle motionMagicExpoDutyCycleRequest;
//     private final MotionMagicExpoVoltage motionMagicExpoVoltageRequest;
//     private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentRequest;

//     public final NeutralOut neutralOutputRequest;

//     private final double driveRotorRotationsPerMeter;
//     private final double maxDriveVelRotPerSec;

//     public BreakerSwerveModule2(CANDeviceID driveMotorID, CANDeviceID azimuthMotorID, CANDeviceID azimuthEncoderID, BreakerSwerveModuleConfig config)  {
//         this.config = config;
//         driveMotor = new TalonFX(driveMotorID.getDeviceID(), driveMotorID.getBusName());
//         azimuthMotor = new TalonFX(azimuthMotorID.getDeviceID(), azimuthMotorID.getBusName());
//         azimuthEncoder = new CANcoder(azimuthEncoderID.getDeviceID(), azimuthEncoderID.getBusName());

//         driveRotorRotationsPerMeter = config.getDriveMotorConfig().getRotorToWheelRatio().getRatioToOne() / (2.0 * Math.PI *  config.getDriveMotorConfig().getWheelRadius().in(Units.Meters));
//         maxDriveVelRotPerSec = config.getDriveMotorConfig().getMaxVelocity().magnitude() * driveRotorRotationsPerMeter;
//     }

//     public BreakerSwerveModuleState getState(boolean refresh, boolean compensateForLatency) {
//         if (refresh) {
//             refreshAllSignals();
//         }
//         double drivePos = drivePositionSignal.getValueAsDouble();
//         double azimuthPos = azimuthPositionSignal.getValueAsDouble();
//         if (compensateForLatency) {
//             drivePos =  BaseStatusSignal.getLatencyCompensatedValue(drivePositionSignal, driveVelocitySignal);
//             azimuthPos = BaseStatusSignal.getLatencyCompensatedValue(azimuthPositionSignal, azimuthVelocitySignal);
//         }
//         return new BreakerSwerveModuleState(
//             Units.Meters.of(drivePos / driveRotorRotationsPerMeter), 
//             Units.MetersPerSecond.of(driveVelocitySignal.getValueAsDouble() / driveRotorRotationsPerMeter), 
//             Rotation2d.fromRotations(azimuthPos), 
//             Units.RotationsPerSecond.of(azimuthVelocitySignal.getValueAsDouble())
//         );
//     } 

//     private void configDriveMotor() {
//         TalonFXConfiguration driveConfig = new TalonFXConfiguration();
//         driveConfig.Slot0.kP = config.getDriveMotorConfig().getClosedLoopControlConfig().getP();
//         driveConfig.Slot0.kI = config.getDriveMotorConfig().getClosedLoopControlConfig().getI();
//         driveConfig.Slot0.kD = config.getDriveMotorConfig().getClosedLoopControlConfig().getD();
//         driveConfig.Slot0.kV = config.getDriveMotorConfig().getClosedLoopControlConfig().getV();
//         driveConfig.Slot0.kA = config.getDriveMotorConfig().getClosedLoopControlConfig().getA();
//         driveConfig.Slot0.kS = config.getDriveMotorConfig().getClosedLoopControlConfig().getS();
//     }

//     public BaseStatusSignal[] getSignals() {
//         return signals;
//     }

//     public void refreshAllSignals() {
//         BaseStatusSignal.refreshAll(signals);
//     }

//     public void setTargetState(SwerveModuleState state, boolean isDriveOpenLoop, boolean optimize, boolean refreshRefrencePosition) {
//         BreakerSwerveModuleState curState = getState(refreshRefrencePosition, true);
//         targetState = state;
//         if (optimize) {
//             targetState = SwerveModuleState.optimize(targetState, curState.azimuthAngle); 
//         }
//         double angleToSet = targetState.angle.getRotations();

//         switch(config.getAzimuthMotorConfig().getClosedLoopControlConfig().getControlType()) {
//             case EXPONENTIAL_DUTY_CYCLE:
//                 break;
//             case EXPONENTIAL_TORQUE_CURRENT:
//                 break;
//             case EXPONENTIAL_VOLTAGE:
//                 break;
//             case TRAPIZODAL_DUTY_CYCLE:
//                 break;
//             case TRAPIZODAL_TORQUE_CURRENT:
//                 break;
//             case TRAPIZODAL_VOLTAGE:
//                 break;
//             default:
//                 break;
            
//         }

//         double velocityToSet = targetState.speedMetersPerSecond * driveRotorRotationsPerMeter;
//         double steerMotorError = angleToSet - curState.azimuthAngle.getRotations();
//         double cosineScalar = Math.cos(steerMotorError * 2.0 * Math.PI);
//         if (cosineScalar < 0.0) {
//             cosineScalar = 0.0;
//         }
//         velocityToSet *= cosineScalar;
//         double azimuthTurnRps = azimuthVelocitySignal.getValue();
//         double driveRateBackOut = azimuthTurnRps * config.getCouplingRatio().getRatioToOne();
//         velocityToSet += driveRateBackOut;
//         velocityToSet = MathUtil.clamp(velocityToSet, -maxDriveVelRotPerSec, maxDriveVelRotPerSec);

//         if (!isDriveOpenLoop) {
//             switch (config.getDriveMotorConfig().getClosedLoopControlConfig().getControlType()) {
//                 case DUTY_CYCLE:
//                     driveMotor.setControl(velocityDutyCycleRequest.withVelocity(velocityToSet))
//                     break;
//                 case TORQUE_CURRENT:
//                     if (!MathUtil.isNear(0.0, velocityToSet, 1E-9)) {
//                         driveMotor.setControl(velocityTorqueCurrentRequest.withVelocity(velocityToSet));
//                     } else {
//                         driveMotor.setControl(neutralOutputRequest);
//                     }
//                     break;
//                 default:
//                 case VOLTAGE:
//                     driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityToSet));
//                     break;
//             } 
//         } else {

//         }
//     }

    

    
// }
