// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.shooter;

// import frc.robot.BreakerLib.devices.BreakerGenericLoopedDevice;
// import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
// import frc.robot.BreakerLib.util.math.BreakerMath;
// import frc.robot.BreakerLib.util.test.suites.BreakerGenericTestSuiteImplementation;
// //import frc.robot.BreakerLib.util.test.suites.flywheel.BreakerFlywheelTestSuite;

// /** A class representing a robot's shooter flywheel and its assocated controle loop */
// public abstract class BreakerGenericFlywheel extends BreakerGenericLoopedDevice /* implements BreakerGenericTestSuiteImplementation<BreakerFlywheelTestSuite> */ {
//     protected BreakerFlywheelConfig config;
//     protected double flywheelTargetVel = 0;
//    // protected BreakerFlywheelTestSuite testSuite;
//     protected double lastVel = 0;
//     protected double accel = 0;
//     protected double accelTol;
//     protected double velTol;
    

//     public BreakerGenericFlywheel(BreakerFlywheelConfig config) {
//         this.config = config;
//         //testSuite = new BreakerFlywheelTestSuite(this);
//         velTol = config.getVelocityTolerence();
//         accelTol = config.getAcclerationTolerence();
//     }

    
//     /** 
//      * @param flywheelTargetSpeedRPM
//      */
//     public void setFlywheelVelocity(double flywheelTargetVel) {
//         this.flywheelTargetVel = flywheelTargetVel;
//     }

//     public abstract double getFlywheelVelocity();

    
//     /** 
//      * @return double
//      */
//     public double getFlywheelTargetVelocity() {
//         return flywheelTargetVel;
//     }

//     /** sets flywheel speed to 0 RPS */
//     public void stopFlywheel() {
//         setFlywheelVelocity(0);
//         BreakerLog.getInstance().logSuperstructureEvent("flywheel stoped");
//     }


//     protected abstract void applyMotorControl(double targetVel);

    
//     /** 
//      * @return boolean
//      */
//     public boolean flywheelIsAtTargetVel() {
//         return BreakerMath.epsilonEquals(flywheelTargetVel, getFlywheelVelocity(), velTol) && BreakerMath.epsilonEquals(accel, 0, accelTol);
//     }

//     @Override
//     public void periodic() {
//         applyMotorControl(flywheelTargetVel);
//         accel = getFlywheelVelocity() - lastVel;
//         lastVel = getFlywheelVelocity();
//     }

    
//     // /** 
//     //  * @return BreakerFlywheelTestSuite
//     //  */
//     // @Override
//     // public BreakerFlywheelTestSuite getTestSuite() {
//     //     return testSuite;
//     // }
// }
