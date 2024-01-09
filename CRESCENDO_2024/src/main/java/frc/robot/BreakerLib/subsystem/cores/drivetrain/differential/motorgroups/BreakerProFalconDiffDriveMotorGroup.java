// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.Pair;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
// import frc.robot.BreakerLib.util.vendorutil.BreakerPhoenix6Util;

// /** Add your docs here. */
// public class BreakerProFalconDiffDriveMotorGroup extends BreakerDiffDriveMotorGroup {
//     private TalonFX[] motors;
//     public BreakerProFalconDiffDriveMotorGroup(boolean invert, TalonFX... motors) {
//         super(
//         invert, 
//         () -> {return motors[0].getRotorPosition().getValue();}, 
//         () -> {return motors[0].getRotorVelocity().getValue();}, 
//         motors
//         );
//         this.motors = motors;
//     }

//     @Override
//     public void runSelfTest() {
//         faultStr = "";
//         health = DeviceHealth.NOMINAL;

//         StringBuilder work = new StringBuilder();
//         for (TalonFX motor : motors) {
//             Pair<DeviceHealth, String> motorFaultData = BreakerPhoenix6Util.checkMotorFaultsAndConnection(motor);
//             if (motorFaultData.getFirst() != DeviceHealth.NOMINAL) {
//                 health = DeviceHealth.FAULT;
//                 work.append(" MOTOR ID (" + motor.getDeviceID() + ") FAULTS: " + motorFaultData.getSecond());
//             }
//         }
//         faultStr = work.toString();
//     }

//     @Override
//     public void setBrakeMode(boolean isEnabled) {
//         BreakerPhoenix6Util.setBrakeMode(isEnabled, motors);
//     }

//     @Override
//     public void setRotorPosition(double newPosition) {
//         motors[0].setRotorPosition(newPosition);
        
//     }

    


    
// }
