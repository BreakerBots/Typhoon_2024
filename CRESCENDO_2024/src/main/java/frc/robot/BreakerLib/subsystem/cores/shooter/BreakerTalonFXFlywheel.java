// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.shooter;

// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerTalonFXSwerveModuleDriveMotor.TalonFXControlOutputUnits;

// /** Add your docs here. */
// public class BreakerTalonFXFlywheel extends BreakerGenericFlywheel{
//     private TalonFXControlOutputUnits controlOutputUnits;
//     private TalonFX[] motors;
//     private final VelocityDutyCycle dutyCycleRequest;
//     private final VelocityVoltage voltageRequest;
//     private final Follower followerRequest;
//     public BreakerTalonFXFlywheel(BreakerFlywheelConfig config, TalonFXControlOutputUnits controlOutputUnits, TalonFX... motors) {
//         super(config);
//         this.motors = motors;
        
//         dutyCycleRequest = new VelocityDutyCycle(0.0, false, 0.0, 0, false);
//         voltageRequest = new VelocityVoltage(0.0, false, 0.0, 0, false);
//         followerRequest = new Follower(0, false);
//     }

//     @Override
//     public void runSelfTest() {
//         // TODO Auto-generated method stub
        
//     }

//     @Override
//     public double getFlywheelVelocity() {
//         return 0;
//     }

//     @Override
//     protected void applyMotorControl(double targetVel) {
//        switch (controlOutputUnits) {
//             case VOLTAGE:
//                 motors[0].setControl(voltageRequest.withVelocity(targetVel));
//                 break;
//             case DUTY_CYCLE:
//             default:
//                 motors[0].setControl(dutyCycleRequest.withVelocity(targetVel));
//                 break;
//         }
//         for (int i = 1; i < motors.length; i++) {
//             motors[i].setControl(followerRequest.withOpposeMasterDirection(motors[i].getInverted()));
//         }
//     }

// }
