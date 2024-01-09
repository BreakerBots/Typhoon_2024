// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.legacy;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.math.Pair;
// import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDriveConfig;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
// import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix5Util;

// /** A {@link BreakerLegacyDiffDrive} instance with TalonFX (Falcon 500) motors */
// @Deprecated
// public class BreakerLegacyFalconDiffDrive extends BreakerLegacyDiffDrive {
//     private WPI_TalonFX[] leftMotors;
//     private WPI_TalonFX[] rightMotors;

//      /** Creates a new Differential (tank drive) drivetrain instance.
//      * 
//      * @param leftMotors Left {@link WPI_TalonFX} motors.
//      * @param invertL Invert left motor outputs & encoder readings.
//      * @param rightMotors Right {@link WPI_TalonFX} motors.
//      * @param invertR Invert right motor outputs & encoder readings.
//      * @param gyro {@link BreakerGenericGyro} capable of reading yaw. 
//      * @param driveConfig A {@link BreakerDiffDriveConfig} representing the configerable values of this drivetrain's kinimatics and control values
//      */
//     public BreakerLegacyFalconDiffDrive(WPI_TalonFX[] leftMotors, WPI_TalonFX[] rightMotors, boolean invertL, boolean invertR,
//         BreakerGenericGyro imu, BreakerDiffDriveConfig driveConfig) {
        
//         super(
//             leftMotors,
//             () -> ((Double)leftMotors[0].getSensorCollection().getIntegratedSensorPosition() / 2048.0),
//             () -> ((Double)((leftMotors[0].getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048.0)),
//             invertL, 
//             rightMotors,
//             () -> ((Double)rightMotors[0].getSensorCollection().getIntegratedSensorPosition() / 2048.0),
//             () -> ((Double)((rightMotors[0].getSensorCollection().getIntegratedSensorVelocity() * 600) / 2048.0)), 
//             invertR,
//             imu,
//             driveConfig);
//     }

//     @Override
//     public void runSelfTest() {
//         faultStr = "";
//         health = DeviceHealth.NOMINAL;

//         StringBuilder work = new StringBuilder();
//         for (WPI_TalonFX motorL : leftMotors) {
//             Pair<DeviceHealth, String> motorFaultData = BreakerPhoenix5Util.checkMotorFaultsAndConnection(motorL);
//             if (motorFaultData.getFirst() != DeviceHealth.NOMINAL) {
//                 health = DeviceHealth.FAULT;
//                 work.append(" MOTOR ID (" + motorL.getDeviceID() + ") FAULTS: " + motorFaultData.getSecond());
//             }
//         }
//         for (WPI_TalonFX motorR : rightMotors) {
//             Pair<DeviceHealth, String> motorFaultData = BreakerPhoenix5Util.checkMotorFaultsAndConnection(motorR);
//             if (motorFaultData.getFirst() != DeviceHealth.NOMINAL) {
//                 health = DeviceHealth.FAULT;
//                 work.append(" MOTOR ID (" + motorR.getDeviceID() + ") FAULTS: " + motorFaultData.getSecond());
//             }
//         }
//         faultStr = work.toString();
        
//     }

//     @Override
//     public void resetDriveEncoders() {
//         leftMotors[0].setSelectedSensorPosition(0);
//         rightMotors[0].setSelectedSensorPosition(0);
//     }

//     @Override
//     public void setDrivetrainBrakeMode(boolean isEnabled) {
//         BreakerPhoenix5Util.setBrakeMode(isEnabled, leftMotors);
//         BreakerPhoenix5Util.setBrakeMode(isEnabled, rightMotors);
//     }

// }
