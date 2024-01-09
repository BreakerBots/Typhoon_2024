// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.util.test.suites.drivetrain.swerve;

// import java.util.ArrayList;
// import java.util.Arrays;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveMovementPreferences;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerGenericSwerveModule;
// import frc.robot.BreakerLib.util.math.averages.BreakerAverage;
// import frc.robot.BreakerLib.util.test.suites.BreakerTestBase;
// import frc.robot.BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;

// /** Stress tests your drivetrain and reports any devations from commanded values */
// public class BreakerSwerveDriveStressTest extends BreakerTestBase {
//     private final Timer timer = new Timer();
//     private BreakerSwerveDrive drivetrain;
//     private double timeoutSeconds;
//     private BreakerAverage[] averageModuleAngleDeltas, averageModuleSpeedDeltas;
//     private ArrayList<ArrayList<Pair<Rotation2d, Double>>> averageModuleDeltas;
//     private ArrayList<Pair<ChassisSpeeds, Double>> speedsToTestAndStepTimeouts;
//     private ArrayList<Pair<ChassisSpeeds, Double>> speedsToTestAndStepTimeoutQueue;
//     private BreakerGenericSwerveModule[] swerveModules;
//     public BreakerSwerveDriveStressTest(BreakerSwerveDrive drivetrain, BreakerGenericSwerveModule[] swerveModules, BreakerTestSuiteDataLogType logType, ArrayList<Pair<ChassisSpeeds, Double>> speedsToTestAndStepTimeouts) {
//         super(logType, " Swerve_Drive_Stress_Test ", " Test Steps And Timeouts: " + speedsToTestAndStepTimeouts.toString());
//         this.drivetrain = drivetrain;
//         this.swerveModules = swerveModules;
//         this.speedsToTestAndStepTimeouts = new ArrayList<>(speedsToTestAndStepTimeouts);
//         this.speedsToTestAndStepTimeoutQueue = new ArrayList<>(speedsToTestAndStepTimeouts);
//         averageModuleAngleDeltas = new BreakerAverage[swerveModules.length];
//         averageModuleSpeedDeltas = new BreakerAverage[swerveModules.length];
//         averageModuleDeltas = new ArrayList<>();
//         for (int i = 0; i < swerveModules.length; i++) {
//            averageModuleAngleDeltas[i] = new BreakerAverage();
//            averageModuleSpeedDeltas[i] = new BreakerAverage();
//         }
//         for (Pair<ChassisSpeeds, Double> step: speedsToTestAndStepTimeouts) {
//             timeoutSeconds += step.getSecond();
//         }
//         addRequirements(drivetrain);
//     }

//     public BreakerSwerveDriveStressTestResult getResult() {
//         return new BreakerSwerveDriveStressTestResult(swerveModules, speedsToTestAndStepTimeouts, averageModuleDeltas);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         timer.start();
//         timer.reset();
//     }

//   // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         ChassisSpeeds setSpeeds = speedsToTestAndStepTimeoutQueue.get(0).getFirst();
//         drivetrain.move(setSpeeds, BreakerSwerveMovementPreferences.DEFAULT_ROBOT_RELATIVE_PREFERENCES.withSlowModeValue(SlowModeValue.DISABLED));
//         SwerveModuleState[] tgtStates = drivetrain.getTargetModuleStates();
//         SwerveModuleState[] resultStates = drivetrain.getSwerveModuleStates();
//         for (int i = 0; i < swerveModules.length; i++) {
//             averageModuleAngleDeltas[i].addValue(tgtStates[i].angle.getRadians() - resultStates[i].angle.getRadians());
//             averageModuleSpeedDeltas[i].addValue(tgtStates[i].speedMetersPerSecond - resultStates[i].speedMetersPerSecond);
//         }
//         if (timer.get() > speedsToTestAndStepTimeoutQueue.get(0).getSecond()) {
//             ArrayList<Pair<Rotation2d, Double>> pairList = new ArrayList<>();
//             for (int i = 0; i < swerveModules.length; i++) {
//                pairList.add(new Pair<Rotation2d, Double>(new Rotation2d(averageModuleAngleDeltas[i].getAverage()), averageModuleSpeedDeltas[i].getAverage()));
//                averageModuleAngleDeltas[i].clear();
//                averageModuleSpeedDeltas[i].clear();
//             }
//             averageModuleDeltas.add(pairList);
//             speedsToTestAndStepTimeoutQueue.remove(0);
//             timer.reset();
//         }
//     }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     logEnd(getResult().toString());
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.get() >= timeoutSeconds || speedsToTestAndStepTimeoutQueue.isEmpty();
//   }

//   public static class BreakerSwerveDriveStressTestResult {
//     private BreakerGenericSwerveModule[] swerveModules;
//     private ArrayList<Pair<ChassisSpeeds, Double>> speedsToTestAndStepTimeouts;
//     private ArrayList<ArrayList<Pair<Rotation2d, Double>>> averageModuleDeltas;
//     public BreakerSwerveDriveStressTestResult(BreakerGenericSwerveModule[] swerveModules, ArrayList<Pair<ChassisSpeeds, Double>> speedsToTestAndStepTimeouts, ArrayList<ArrayList<Pair<Rotation2d, Double>>> averageModuleDeltas) {
//         this.swerveModules = swerveModules;
//         this.averageModuleDeltas = averageModuleDeltas;
//         this.speedsToTestAndStepTimeouts = speedsToTestAndStepTimeouts;
//     }

//     public ArrayList<ArrayList<Pair<Rotation2d, Double>>> getAverageModuleDeltas() {
//         return averageModuleDeltas;
//     }

//     public ArrayList<Pair<ChassisSpeeds, Double>> getSpeedsToTestAndStepTimeouts() {
//         return speedsToTestAndStepTimeouts;
//     }

//     @Override
//     public String toString() {
//         StringBuilder work = new StringBuilder();
//         for (int i = 0; i < speedsToTestAndStepTimeouts.size(); i++) {
//             work.append(String.format("(Target Chassis Speeds: %s, Runtime: %s):\n",speedsToTestAndStepTimeouts.get(i).getFirst().toString(), speedsToTestAndStepTimeouts.get(i).getSecond().toString()));
//             for (int j = 0; j < swerveModules.length; j++) {
//                 Pair<Rotation2d, Double> pair = averageModuleDeltas.get(i).get(j);
//                 work.append(String.format("\t(Module Name: %s) - (Angle Delta: %s) (Speed Delta: %s)\n", swerveModules[j].getDeviceName(), pair.getFirst().toString(), pair.getSecond().toString()));
//             }
//         }
//         return work.toString();
//     }
//   }


// }


