// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.util.test.suites.drivetrain.diff;

// import java.util.ArrayList;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;
// import frc.robot.BreakerLib.util.BreakerTriplet;
// import frc.robot.BreakerLib.util.math.averages.BreakerAverage;
// import frc.robot.BreakerLib.util.test.suites.BreakerTestBase;
// import frc.robot.BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;

// /** Tests the maximum achiveable free velocites of each swerve module's drive motors */
// public class BreakerDiffDriveStressTest extends BreakerTestBase {
//     private Timer timer;
//     private BreakerDiffDrive drivetrain;
//     private double timeoutSeconds;
//     private BreakerAverage leftVelAvg, rightVelAvg;
//     private ArrayList<Pair<Double, Double>> averageDriveVels;
//     private ArrayList<BreakerTriplet<Double, Double, Double>> tankDrivePrecentagesToTestAndStepTimeouts;
//     public BreakerDiffDriveStressTest(BreakerDiffDrive drivetrain, BreakerTestSuiteDataLogType logType, ArrayList<BreakerTriplet<Double, Double, Double>> tankDrivePrecentagesToTestAndStepTimeouts) {
//         super(logType, " Diff_Drive_Stress_Test ", " Test Steps And Timeouts: " + tankDrivePrecentagesToTestAndStepTimeouts.toString());
//         this.drivetrain = drivetrain;
//         tankDrivePrecentagesToTestAndStepTimeouts = new ArrayList<>(tankDrivePrecentagesToTestAndStepTimeouts); 
//         leftVelAvg = new BreakerAverage();
//         rightVelAvg = new BreakerAverage();
//         averageDriveVels = new ArrayList<>();
//         addRequirements(drivetrain);
//     }

//     public BreakerDiffDriveStressTestResult getResult() {
//         return new BreakerDiffDriveStressTestResult(averageDriveVels, tankDrivePrecentagesToTestAndStepTimeouts);
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
//         BreakerTriplet<Double, Double, Double> trip = tankDrivePrecentagesToTestAndStepTimeouts.get(0);
//         drivetrain.tankDrive(trip.getLeft(), trip.getMiddle(), SlowModeValue.DISABLED);
//         DifferentialDriveWheelSpeeds wheelSpeeds = drivetrain.getWheelSpeeds();
//         leftVelAvg.addValue(wheelSpeeds.leftMetersPerSecond);
//         rightVelAvg.addValue(wheelSpeeds.rightMetersPerSecond);
//         if (timer.get() > tankDrivePrecentagesToTestAndStepTimeouts.get(0).getRight()) {
//             averageDriveVels.add(new Pair<Double,Double>(leftVelAvg.getAverage(), rightVelAvg.getAverage()));
//             leftVelAvg.clear();
//             rightVelAvg.clear();
//             tankDrivePrecentagesToTestAndStepTimeouts.remove(0);
//             timer.reset();
//         }
        
//         periodicLog(String.format("Left Speed Precentage: %s | Right Speed Precentage: %s | Left Vel %d | Right Vel %",
//             trip.getLeft().toString(), trip.getMiddle().toString(), wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond));
//     }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     logEnd(getResult().toString());
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.get() >= timeoutSeconds;
//   }

//   public static class BreakerDiffDriveStressTestResult {
//     private ArrayList<Pair<Double, Double>> averageDriveVels;
//     private ArrayList<BreakerTriplet<Double, Double, Double>> tankDrivePrecentagesToTestAndStepTimeouts;
//     public BreakerDiffDriveStressTestResult(ArrayList<Pair<Double, Double>> averageDriveVels, ArrayList<BreakerTriplet<Double, Double, Double>> tankDrivePrecentagesToTestAndStepTimeouts) {
//         this.averageDriveVels = averageDriveVels;
//         this.tankDrivePrecentagesToTestAndStepTimeouts = tankDrivePrecentagesToTestAndStepTimeouts;
//     }

//     public ArrayList<Pair<Double, Double>> getAverageDriveVels() {
//         return averageDriveVels;
//     }

//     public ArrayList<BreakerTriplet<Double, Double, Double>> getTankDrivePrecentagesToTestAndStepTimeouts() {
//         return tankDrivePrecentagesToTestAndStepTimeouts;
//     }

//     @Override
//     public String toString() {
//         StringBuilder work = new StringBuilder();
//         for (int i = 0; i < Math.min(averageDriveVels.size(), tankDrivePrecentagesToTestAndStepTimeouts.size()); i++ ) {
//             BreakerTriplet<Double, Double, Double> trip = tankDrivePrecentagesToTestAndStepTimeouts.get(i);
//             Pair<Double, Double> pair = averageDriveVels.get(i);
//             work.append(String.format("(Left Precent: %s, Right Precent %s, Runtime Sec %s) - (Left Avg Vel m/s: %s, Right Avg Vel m/s: %s)\n", 
//                 trip.getLeft().toString(), trip.getMiddle().toString(), trip.getRight().toString(), pair.getFirst().toString(), pair.getSecond().toString()));
//         }
//         return work.toString();
//     }
//   }


// }


