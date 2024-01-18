// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.position.odometry.differential;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
// import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
// import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;
// import frc.robot.BreakerLib.util.math.BreakerMath;

// public class BreakerDiffDriveOdometer extends SubsystemBase implements BreakerGenericOdometer {
//     private DifferentialDriveOdometry odometry;
//     private BreakerDiffDrive drivetrain;
//     private BreakerMovementState2d prevMovementState = new BreakerMovementState2d(),
//       curMovementState = new BreakerMovementState2d();
//     private double prevOdometryUpdateTimestamp = 0;
//     public BreakerDiffDriveOdometer(BreakerDiffDrive drivetrain, Pose2d initialPose) {
//         this.drivetrain = drivetrain;
//         odometry = new DifferentialDriveOdometry(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getLeftDriveWheelDistance(), drivetrain.getRightDriveWheelDistance(), initialPose);
//     }

//     public BreakerDiffDriveOdometer(BreakerDiffDrive drivetrain) {
//        this(drivetrain, new Pose2d());
//     }

//     @Override
//     public void setOdometryPosition(Pose2d newPose) {
//       odometry.resetPosition(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getLeftDriveWheelDistance(), drivetrain.getRightDriveWheelDistance(), newPose);
//     }

//     @Override
//     public Pose2d getOdometryPoseMeters() {
//       return odometry.getPoseMeters();
//     }

//     @Override
//     public BreakerMovementState2d getMovementState() {
//       return curMovementState;
//     }

//     @Override
//     public ChassisSpeeds getRobotRelativeChassisSpeeds() {
//       return drivetrain.getKinematics().toChassisSpeeds(drivetrain.getWheelSpeeds());
//     }
  
//     @Override
//     public ChassisSpeeds getFieldRelativeChassisSpeeds() {
//       return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), drivetrain.getOdometryPoseMeters().getRotation());
//     }

//     private void calculateMovementState(double timeToLastUpdateMiliseconds) {
//         curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(),
//             getFieldRelativeChassisSpeeds(), timeToLastUpdateMiliseconds, prevMovementState);
//         prevMovementState = curMovementState;
//     }

//     @Override
//     public void periodic() {
//         odometry.update(drivetrain.getBaseGyro().getYawRotation2d(), drivetrain.getLeftDriveWheelDistance(), drivetrain.getRightDriveWheelDistance());
//         calculateMovementState((Timer.getFPGATimestamp() - prevOdometryUpdateTimestamp) * 1000);
//         prevOdometryUpdateTimestamp = Timer.getFPGATimestamp();
//     }
    
//   }
