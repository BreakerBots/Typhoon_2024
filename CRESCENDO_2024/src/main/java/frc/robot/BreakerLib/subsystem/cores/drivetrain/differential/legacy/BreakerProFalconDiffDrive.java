// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.legacy;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.Pair;
// import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDriveConfig;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
// import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;
// import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

// /** A {@link BreakerLegacyDiffDrive} instance with TalonFX (Falcon 500) motors running Phoenix Pro firmware */
// public class BreakerProFalconDiffDrive extends BreakerLegacyDiffDrive {
//     private TalonFX[] leftMotors;
//     private TalonFX[] rightMotors;

//      /** Creates a new Differential (tank drive) drivetrain instance.
//      * 
//      * @param leftMotors Left {@link TalonFX} motors.
//      * @param invertL Invert left motor outputs & encoder readings.
//      * @param rightMotors Right {@link TalonFX} motors.
//      * @param invertR Invert right motor outputs & encoder readings.
//      * @param gyro {@link BreakerGenericGyro} capable of reading yaw. 
//      * @param driveConfig A {@link BreakerDiffDriveConfig} representing the configerable values of this drivetrain's kinimatics and control values
//      */
//     public BreakerProFalconDiffDrive(TalonFX[] leftMotors, TalonFX[] rightMotors, boolean invertL, boolean invertR,
//         BreakerGenericGyro imu, BreakerDiffDriveConfig driveConfig) {
        
//         super(
//             leftMotors,
//             () -> {return leftMotors[0].getRotorPosition().getValue();},
//             () -> {return leftMotors[0].getRotorVelocity().getValue();},
//             invertL, 
//             rightMotors,
//             () -> {return rightMotors[0].getRotorPosition().getValue();},
//             () -> {return rightMotors[0].getRotorVelocity().getValue();},
//             invertR,
//             imu,
//             driveConfig);
//     }

//     @Override
//     public void runSelfTest() {
//         faultStr = "";
//         health = DeviceHealth.NOMINAL;

//         StringBuilder work = new StringBuilder();
//         for (TalonFX motorL : leftMotors) {
//             Pair<DeviceHealth, String> motorFaultData = BreakerPhoenix6Util.checkMotorFaultsAndConnection(motorL);
//             if (motorFaultData.getFirst() != DeviceHealth.NOMINAL) {
//                 health = DeviceHealth.FAULT;
//                 work.append(" MOTOR ID (" + motorL.getDeviceID() + ") FAULTS: " + motorFaultData.getSecond());
//             }
//         }
//         for (TalonFX motorR : rightMotors) {
//             Pair<DeviceHealth, String> motorFaultData = BreakerPhoenix6Util.checkMotorFaultsAndConnection(motorR);
//             if (motorFaultData.getFirst() != DeviceHealth.NOMINAL) {
//                 health = DeviceHealth.FAULT;
//                 work.append(" MOTOR ID (" + motorR.getDeviceID() + ") FAULTS: " + motorFaultData.getSecond());
//             }
//         }
//         faultStr = work.toString();
        
//     }

//     @Override
//     public void resetDriveEncoders() {
//         leftMotors[0].setRotorPosition(0);
//         rightMotors[0].setRotorPosition(0);
//     }

//     @Override
//     public void setDrivetrainBrakeMode(boolean isEnabled) {
//         BreakerPhoenix6Util.setBrakeMode(isEnabled, leftMotors);
//         BreakerPhoenix6Util.setBrakeMode(isEnabled, rightMotors);
//     }

// }
