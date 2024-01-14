// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.util.test.suites.drivetrain.swerve;

// import java.util.ArrayList;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.LegacyBreakerSwerveDrive;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
// import frc.robot.BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;

// /** Add your docs here. */
// public class BreakerSwerveDriveTestSuite {
//     private LegacyBreakerSwerveDrive drivetrain;
//     private BreakerTestSuiteDataLogType logType;
//     private BreakerGenericSwerveModule[] modules;
//     public BreakerSwerveDriveTestSuite(LegacyBreakerSwerveDrive drivetrain, BreakerGenericSwerveModule... modules) {
//         this.drivetrain = drivetrain;
//         this.modules = modules;
//         logType = BreakerTestSuiteDataLogType.PARTIAL_AUTOLOG;
//     }

//     public void setLogType(BreakerTestSuiteDataLogType newLogType) {
//         logType = newLogType;
//     }

//     public BreakerSwerveModuleVelocityLimitTest moduleVelocityLimitTest(double testTimeSeconds, double targetMaxSpeed) {
//         BreakerSwerveModuleVelocityLimitTest test = new BreakerSwerveModuleVelocityLimitTest(drivetrain, modules, targetMaxSpeed, testTimeSeconds, logType);
//         return test;
//     }

//     /** Creates a standard swerve drive stress test (Total: 32 seconds)<br><br>
//      * 
//      * 1. (X: 100%, Y: 0%, Angular: 0%) for 1.75 seconds<br>
//      * 2. (X: -100%, Y: 0%, Angular: 0%) for 1.75 seconds<br>
//      * 3. (X: 0%, Y: 100%, Angular: 0%) for 1.75 seconds<br>
//      * 4. (X: 0%, Y: -100%, Angular: 0%) for 1.75 seconds<br>
//      * 5. (X: 0%, Y: 0%, Angular: 100%) for 1.75 seconds<br>
//      * 6. (X: 0%, Y: 0%, Angular: -100%) for 1.75 seconds<br>
//      * 7. (X: 100%, Y: 0%, Angular: 100%) for 1.75 seconds<br>
//      * 8. (X: 0%, Y: 100%, Angular: 100%) for 1.75 seconds<br>
//      * 
//      * 9. (X: 25%, Y: 0%, Angular: 0%) for 1.75 seconds<br>
//      * 10. (X: -25%, Y: 0%, Angular: 0%) for 1.75 seconds<br>
//      * 11. (X: 0%, Y: 25%, Angular: 0%) for 1.75 seconds<br>
//      * 12. (X: 0%, Y: -25%, Angular: 0%) for 1.75 seconds<br>
//      * 13. (X: 0%, Y: 0%, Angular: 25%) for 1.75 seconds<br>
//      * 14. (X: 0%, Y: 0%, Angular: -25%) for 1.75 seconds<br>
//      * 15. (X: 25%, Y: 0%, Angular: 25%) for 1.75 seconds<br>
//      * 16. (X: 0%, Y: 25%, Angular: 25%) for 1.75 seconds<br>
//      * 
//      * 17. (X: 100%, Y: 100%, Angular: 0%) for 0.5 seconds<br>
//      * 18. (X: 100%, Y: -100%, Angular: 0%) for 0.5 seconds<br>
//      * 19. (X: -100%, Y: -100%, Angular: 0%) for 0.5 seconds<br>
//      * 20. (X: -100%, Y: 100%, Angular: 0%) for 0.5 seconds<br>
//      * 
//      * 21. (X: 100%, Y: 100%, Angular: 0%) for 0.5 seconds<br>
//      * 22. (X: 100%, Y: -100%, Angular: 0%) for 0.5 seconds<br>
//      * 23. (X: -100%, Y: -100%, Angular: 0%) for 0.5 seconds<br>
//      * 24. (X: -100%, Y: 100%, Angular: 0%) for 0.5 seconds<br>
//      */
//     // public BreakerSwerveDriveStressTest stressTest(double maxVelX, double maxVelY, double maxVelOmega) {
//     //     ArrayList<Pair<ChassisSpeeds, Double>> speedList = new ArrayList<>();

//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(maxVelX, 0, 0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(-maxVelX, 0, 0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, maxVelY, 0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, -maxVelY, 0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, 0, maxVelOmega), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, 0, -maxVelOmega), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(maxVelX, 0, maxVelOmega), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, maxVelY, maxVelOmega), 1.75));

//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(maxVelX/4.0, 0, 0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(-maxVelX/4.0, 0, 0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, maxVelY/4.0, 0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, -maxVelY/4.0, 0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, 0, maxVelOmega/4.0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, 0, -maxVelOmega/4.0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(maxVelX/4.0, 0, maxVelOmega/4.0), 1.75));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(0, maxVelY/4.0, maxVelOmega/4.0), 1.75));

//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(maxVelX, maxVelY, 0), 0.5));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(maxVelX, -maxVelY, 0), 0.5));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(-maxVelX, -maxVelY, 0), 0.5));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(-maxVelX, maxVelY, 0), 0.5));

//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(maxVelX, maxVelY, 0), 0.5));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(maxVelX, -maxVelY, 0), 0.5));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(-maxVelX, -maxVelY, 0), 0.5));
//     //     speedList.add(new Pair<ChassisSpeeds,Double>(new ChassisSpeeds(-maxVelX, maxVelY, 0), 0.5));
       
//     //     BreakerSwerveDriveStressTest test = new BreakerSwerveDriveStressTest(drivetrain, modules, logType, speedList);
//     //     return test;
//     // }

//     // public BreakerSwerveDriveStressTest stressTest(ArrayList<Pair<ChassisSpeeds, Double>> chassisSpeedsAndCutoffTimes) {
//     //     ArrayList<Pair<ChassisSpeeds, Double>> speedList = new ArrayList<>(chassisSpeedsAndCutoffTimes);
//     //     BreakerSwerveDriveStressTest test = new BreakerSwerveDriveStressTest(drivetrain, modules, logType, speedList);
//     //     return test;
//     // }




// }