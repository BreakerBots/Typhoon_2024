// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import static frc.robot.Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
// import static frc.robot.Constants.VisionConstants.BACK_CAMERA_NAME;
// import static frc.robot.Constants.VisionConstants.BACK_CAMERA_TRANS;
// import static frc.robot.Constants.VisionConstants.LIMELIGHT_NAME;
// import static frc.robot.Constants.VisionConstants.LIMELIGHT_TRANS;

// import java.util.List;

// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.BreakerLib.devices.vision.limelight.BreakerLimelight;
// import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
// import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera.BreakerPhotonVisionPoseEstimator;
// import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerPoseEstimationStandardDevationCalculator;
// import frc.robot.subsystems.Drive;

// /** Add your docs here. */
// public class Vision extends SubsystemBase {
//     public BreakerLimelight limelight;
//     public BreakerPhotonCamera frontCam, backCam;
//     private BreakerPhotonVisionPoseEstimator backPosSrc;
//     private Drive drivetrain;
//    // private PhotonPoseEstimator poseEst;

//     public Vision(Drive drivetrain) {
//         limelight = new BreakerLimelight(LIMELIGHT_NAME, LIMELIGHT_TRANS);
//         //frontCam = new BreakerPhotonCamera(FRONT_CAMERA_NAME, FRONT_CAMERA_TRANS);
//         backCam = new BreakerPhotonCamera(BACK_CAMERA_NAME, BACK_CAMERA_TRANS);
//         backPosSrc = backCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDevationCalculator());
//         this.drivetrain = drivetrain;
//         drivetrain.getOdometryThread().registerEstimatedPoseSource(backPosSrc);
//     }

//     public List<PhotonTrackedTarget> getFrontCamTrackedTargets() {
//         return null;
//     }

//     public List<PhotonTrackedTarget> getBackCamTrackedTargets() {
//         return backPosSrc.getTrackedTargets();
//     }

//     @Override
//     public void periodic() {
//         // Optional<EstimatedRobotPose> posOpt = poseEst.update();
//         // if (posOpt.isPresent()) {
//         //     BreakerLog.recordOutput("NewVisPos", Pose2d.struct, posOpt.get().estimatedPose.toPose2d());
//         // }
        
//         // Optional<BreakerEstimatedPose> estPosOpt = backPosSrc.getEstimatedPose(PoseOrigin.ofGlobal());
//         // if (estPosOpt.isPresent()) {
//         //     BreakerEstimatedPose estPos = estPosOpt.get();
//         //     BreakerLog.recordOutput("pose", Pose2d.struct, estPos.estimatedPose.toPose2d());
//         //     Matrix<N3, N1> devMatrix = estPos.estimationStandardDevations.get();
//         //     BreakerLog.recordOutput("StdDevs", new double[]{devMatrix.get(0, 0), devMatrix.get(1, 0), devMatrix.get(2, 0)});
//         // }
        
//     }
// }
