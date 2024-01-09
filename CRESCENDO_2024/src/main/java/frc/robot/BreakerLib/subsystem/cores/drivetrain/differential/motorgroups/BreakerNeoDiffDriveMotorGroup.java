// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.motorgroups;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.CANSparkMax;

// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
// import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

// /** Add your docs here. */
// public class BreakerNeoDiffDriveMotorGroup extends BreakerDiffDriveMotorGroup {
//     private CANSparkMax[] motors;
//     public BreakerNeoDiffDriveMotorGroup(boolean invert, CANSparkMax... motors) {
//         super(
//         invert, 
//         () -> {return motors[0].getEncoder().getPosition();}, 
//         () -> {return motors[0].getEncoder().getVelocity() / 60.0;}, 
//         motors
//         );
//         this.motors = motors;
//     }

//     @Override
//     public void runSelfTest() {
//         faultStr = "";
//         health = DeviceHealth.NOMINAL;

//         StringBuilder work = new StringBuilder();
//         for (CANSparkMax motorL : motors) {
//             short faults = motorL.getFaults();
//             if ((int) faults != 0) {
//                 health = DeviceHealth.FAULT;
//                 work.append(" MOTOR ID (" + motorL.getDeviceId() + ") FAULTS: ");
//                 work.append(BreakerREVUtil.getSparkMaxHealthAndFaults(faults).getSecond());
//             }
//         }
//         faultStr = work.toString();
        
//     }

//     @Override
//     public void setBrakeMode(boolean isEnabled) {
//         BreakerREVUtil.setBrakeMode(isEnabled, motors);
//     }

//     @Override
//     public void setRotorPosition(double newPosition) {
//         motors[0].getEncoder().setPosition(newPosition);
//     }



    

    


    
// }
