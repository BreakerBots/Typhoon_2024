// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.position.odometry.differential;

// import edu.wpi.first.math.MatBuilder;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
// import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;
// import frc.robot.BreakerLib.util.math.BreakerMath;

// /** Add your docs here. */
// public class BreakerDiffDrivePoseEstimator extends SubsystemBase implements BreakerGenericOdometer {

//     private DifferentialDrivePoseEstimator poseEstimator;
//     private BreakerDiffDrive drivetrain;
//     private Pose2d currentPose;
//     private double lastUpdateTimestamp = Timer.getFPGATimestamp();
//     private Pose2d prevPose = getOdometryPoseMeters();
//     private ChassisSpeeds fieldRelativeChassisSpeeds = new ChassisSpeeds();
//     private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
//     private BreakerMovementState2d curMovementState = new BreakerMovementState2d();

//     public BreakerDiffDrivePoseEstimator(BreakerDiffDrive drivetrain, Pose2d initialPose,
//             double[] stateStanderdDeviation,
//             double[] visionStanderdDeviation) {
        
//         this.drivetrain = drivetrain;
//         currentPose = initialPose;
//         poseEstimator = new DifferentialDrivePoseEstimator(
//         drivetrain.getKinematics(), drivetrain.getBaseGyro().getYawRotation2d(),
//         drivetrain.getLeftDriveWheelDistance(),
//         drivetrain.getLeftDriveWheelDistance(),
//         initialPose,
//         new MatBuilder<>(Nat.N3(), Nat.N1()).fill(stateStanderdDeviation),
//         new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStanderdDeviation));
      
//     }

//     public Pose2d addVisionMeasurment(Pose2d robotPoseFromVision, double visionDataTimestamp) {
//         poseEstimator.addVisionMeasurement(robotPoseFromVision, visionDataTimestamp);
//         currentPose = poseEstimator.getEstimatedPosition();
//         return currentPose;
//     }

//     public void changeVisionDevs(double visionStrdDevX, double visionStrdDevY, double visionStrdDevTheta) {
//         poseEstimator.setVisionMeasurementStdDevs(
//                 new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStrdDevX, visionStrdDevY, visionStrdDevTheta));
//     }

//     public void setOdometryPosition(Pose2d newPose) {
//         poseEstimator.resetPosition(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getRightDriveWheelDistance(), drivetrain.getRightDriveWheelDistance(), newPose);
//     }

//     @Override
//     public String toString() {
//         return "CURRENT POSE: " + getOdometryPoseMeters().toString();
//     }

//     @Override
//     public Pose2d getOdometryPoseMeters() {
//         currentPose = poseEstimator.getEstimatedPosition();
//         return currentPose;
//     }

//     @Override
//     public BreakerMovementState2d getMovementState() {
//         return curMovementState;
//     }

//     @Override
//     public ChassisSpeeds getRobotRelativeChassisSpeeds() {
//         return ChassisSpeeds.fromFieldRelativeSpeeds(
//                 fieldRelativeChassisSpeeds.vxMetersPerSecond,
//                 fieldRelativeChassisSpeeds.vyMetersPerSecond,
//                 fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
//                 getOdometryPoseMeters().getRotation());
//     }

//     @Override
//     public ChassisSpeeds getFieldRelativeChassisSpeeds() {
//         return fieldRelativeChassisSpeeds;
//     }

//     private void updateChassisSpeeds() {
//         double timeDiff = Timer.getFPGATimestamp() - lastUpdateTimestamp;
//         double xSpeed = (getOdometryPoseMeters().getX() - prevPose.getX()) * timeDiff;
//         double ySpeed = (getOdometryPoseMeters().getY() - prevPose.getY()) * timeDiff;
//         double thetaSpeed = (getOdometryPoseMeters().getRotation().getRadians() - prevPose.getRotation().getRadians())
//                 * timeDiff;
//         fieldRelativeChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
//         curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(),
//                 fieldRelativeChassisSpeeds, timeDiff, prevMovementState);
//     }

//     @Override
//     public void periodic() {
//         prevPose = getOdometryPoseMeters();
//         currentPose = poseEstimator.update(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getLeftDriveWheelDistance(),
//         drivetrain.getRightDriveWheelDistance());
//         updateChassisSpeeds();
//         lastUpdateTimestamp = Timer.getFPGATimestamp();
//     }

// }
