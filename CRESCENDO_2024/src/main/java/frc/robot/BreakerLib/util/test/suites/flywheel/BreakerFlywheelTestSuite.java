// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.util.test.suites.flywheel;

// import frc.robot.BreakerLib.subsystem.cores.shooter.BreakerGenericFlywheel;
// import frc.robot.BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;

// /** Add your docs here. */
// public class BreakerFlywheelTestSuite {
//     private BreakerGenericFlywheel baseFlywheel;
//     private BreakerTestSuiteDataLogType logType;
//     public BreakerFlywheelTestSuite(BreakerGenericFlywheel baseFlywheel) {
//         this.baseFlywheel = baseFlywheel;
//         logType = BreakerTestSuiteDataLogType.PARTIAL_AUTOLOG;
//     }

//     public void setLogType(BreakerTestSuiteDataLogType newLogType) {
//         logType = newLogType;
//     }

//     public BreakerTimedFlywheelSpinUpTest timedSpinUpTest(double testTimeSeconds, double targetRPM) {
//         BreakerTimedFlywheelSpinUpTest test = new BreakerTimedFlywheelSpinUpTest(testTimeSeconds, targetRPM, baseFlywheel, logType);
//         return test;
//     }

//     public BreakerRepeatedFlywheelChargeCycleTest repeatedChargeCycleTest(int numberOfCycles, double targetRPM) {
//         BreakerRepeatedFlywheelChargeCycleTest test = new BreakerRepeatedFlywheelChargeCycleTest(numberOfCycles, targetRPM, baseFlywheel, logType);
//         return test;
//     }

// }
