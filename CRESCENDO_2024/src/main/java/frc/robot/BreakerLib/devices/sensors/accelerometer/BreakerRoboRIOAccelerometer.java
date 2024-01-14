// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.devices.sensors.accelerometer;

// import edu.wpi.first.wpilibj.BuiltInAccelerometer;

// /** RoboRIO accelerometer using the {@link BreakerGenericAccelerometer} interface. */
// public class BreakerRoboRIOAccelerometer implements BreakerGenericAccelerometer{

//     private BuiltInAccelerometer accelerometer;
    
//     /** RoboRIO accelerometer with range of 8 Gs. */
//     public BreakerRoboRIOAccelerometer() {
//         accelerometer = new BuiltInAccelerometer();
//     }

//     @Override
//     public double[] getRawAccelerometerVals() {
//         return new double[] {accelerometer.getX(), accelerometer.getY(), accelerometer.getZ()};
//     }

//     @Override
//     public double getRawAccelX() {
//         return accelerometer.getX();
//     }

//     @Override
//     public double getRawAccelY() {
//         return accelerometer.getY();
//     }

//     @Override
//     public double getRawAccelZ() {
//         return accelerometer.getZ();
//     }
// }
