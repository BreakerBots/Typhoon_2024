// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.rev;

// import java.util.Optional;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMax.IdleMode;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.util.Units;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
// import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
// import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

// /** Add your docs here. */
// public class BreakerBrushlessSparkMaxSwerveModuleDriveMotor extends BreakerGenericSwerveModuleDriveMotor {
//     private CANSparkMax motor;
//     private double driveGearRatio, wheelDiameter, targetVelocity;
//     private BreakerArbitraryFeedforwardProvider arbFF;
//     private BreakerSwerveModuleDriveMotorConfig config;
//     public BreakerBrushlessSparkMaxSwerveModuleDriveMotor(CANSparkMax motor, boolean isMotorInverted, BreakerSwerveModuleDriveMotorConfig config) {
//         this.motor = motor;
//         this.config = config;
//         this.driveGearRatio = config.getDriveGearRatio();
//         this.wheelDiameter = config.getWheelDiameter();
//         this.arbFF = config.getArbFF();
//         motor.restoreFactoryDefaults();
//         targetVelocity = 0.0;
//         SparkMaxPIDController drivePID = motor.getPIDController();
//         BreakerREVUtil.checkError(drivePID.setP(config.getPIDConfig().kP), "Failed to config " + deviceName + " closed loop kP");
//         BreakerREVUtil.checkError(drivePID.setP(config.getPIDConfig().kI), "Failed to config " + deviceName + " closed loop kI");
//         BreakerREVUtil.checkError(drivePID.setP(config.getPIDConfig().kD), "Failed to config " + deviceName + " closed loop kD");
//         BreakerREVUtil.checkError(drivePID.setP(config.getPIDConfig().kF), "Failed to config " + deviceName + " closed loop kF");

//         Optional<Double> outputRampPeriod = config.getOutputRampPeriod();
//         if (outputRampPeriod.isPresent()) {
//             BreakerREVUtil.checkError(motor.setClosedLoopRampRate(outputRampPeriod.get()), "Failed to config " + deviceName + " closed loop ramp rate");
//             BreakerREVUtil.checkError(motor.setOpenLoopRampRate(outputRampPeriod.get()), "Failed to config " + deviceName + " open loop ramp rate");
//         }

//         BreakerREVUtil.checkError(motor.enableVoltageCompensation(12.0), "Failed to config " + deviceName + " voltage compensation");
//         BreakerREVUtil.checkError(motor.setSmartCurrentLimit((int) config.getSupplyCurrentLimit()),  "Failed to config " + deviceName + " smart current limit");
//         motor.setInverted(isMotorInverted);
//         motor.setIdleMode(IdleMode.kBrake);
//         motor.burnFlash();
//     }

//     @Override
//     public void runSelfTest() {
//         faultStr = "";
//         Pair<DeviceHealth, String> pair = BreakerREVUtil.getSparkMaxHealthAndFaults(motor.getFaults());
//         health = pair.getFirst();
//         if (health != DeviceHealth.NOMINAL) {
//             faultStr = " DRIVE_MOTOR_FAULTS : " + pair.getSecond();
//         }
//     }

//     @Override
//     public void setTargetVelocity(double targetMetersPerSecond, boolean isOpenLoop) {
//         targetVelocity = targetMetersPerSecond;
//         if (isOpenLoop) {
//             motor.getPIDController().setReference(targetMetersPerSecond / config.getMaxAttainableWheelSpeed(), ControlType.kDutyCycle);
//         } else {
//             motor.getPIDController().setReference(getMetersPerSecToNativeVelUnits(targetMetersPerSecond),
//                 CANSparkMax.ControlType.kVelocity, 0, arbFF.getArbitraryFeedforwardValue(targetMetersPerSecond),
//                 SparkMaxPIDController.ArbFFUnits.kPercentOut);
//         }
        
//     }

//     @Override
//     public double getVelocity() {
//         return ((motor.getEncoder().getVelocity() / driveGearRatio) * (wheelDiameter * Math.PI)) / 60.0;
//     }

//     @Override
//     public double getDistance() {
//         return ((motor.getEncoder().getPosition() / driveGearRatio) * (wheelDiameter * Math.PI)) / 60.0;
//     }

//     @Override
//     public void resetDistance() {
//         motor.getEncoder().setPosition(0);
//     }

//     @Override
//     public void setBrakeMode(boolean isEnabled) {
//         motor.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
//     }

//     @Override
//     public double getTargetVelocity() {
//         return targetVelocity;
//     }

//     private double getMetersPerSecToNativeVelUnits(double speedMetersPerSec) {
//         return ((speedMetersPerSec * 60.0) * driveGearRatio) / (wheelDiameter * Math.PI);
//     }

//     @Override
//     public double getSupplyCurrent() {
//         return motor.getOutputCurrent();
//     }

//     @Override
//     public double getMotorOutput() {
//         return motor.getAppliedOutput();
//     }

//     @Override
//     public BreakerSwerveModuleDriveMotorConfig getConfig() {
//         return config;
//     }
// }
