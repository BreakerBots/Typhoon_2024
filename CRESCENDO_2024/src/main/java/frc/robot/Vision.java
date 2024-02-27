// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.Constants.VisionConstants.BACK_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.BACK_CAMERA_TRANS;
import static frc.robot.Constants.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.FRONT_CAMERA_TRANS;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_NAME;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_TRANS;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.limelight.BreakerLimelight;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera.BreakerPhotonVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerPoseEstimationStandardDevationCalculator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.subsystems.Drive;

/** Add your docs here. */
public class Vision extends SubsystemBase {
    public BreakerLimelight limelight;
    public BreakerPhotonCamera frontCam, leftCam, rightCam, backCam;
    private BreakerPhotonVisionPoseEstimator frontPosSrc, leftPosSrc, rightPosSrc, backPosSrc;
    private Drive drivetrain;
   // private PhotonPoseEstimator poseEst;

    public Vision(Drive drivetrain) {
        limelight = new BreakerLimelight(LIMELIGHT_NAME, LIMELIGHT_TRANS);
        frontCam = new BreakerPhotonCamera(FRONT_CAMERA_NAME, FRONT_CAMERA_TRANS);
        // leftCam = new BreakerPhotonCamera(LEFT_CAMERA_NAME, LEFT_CAMERA_TRANS);
        // rightCam = new BreakerPhotonCamera(RIGHT_CAMERA_NAME, RIGHT_CAMERA_TRANS);
        backCam = new BreakerPhotonCamera(BACK_CAMERA_NAME, BACK_CAMERA_TRANS);

        frontPosSrc = frontCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDevationCalculator());
        // leftPosSrc = leftCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDevationCalculator());
        // rightPosSrc = rightCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDevationCalculator());
        backPosSrc = backCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDevationCalculator());
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Optional<EstimatedRobotPose> posOpt = poseEst.update();
        // if (posOpt.isPresent()) {
        //     BreakerLog.recordOutput("NewVisPos", Pose2d.struct, posOpt.get().estimatedPose.toPose2d());
        // }

        Optional<BreakerEstimatedPose> frontPosOpt = frontPosSrc.getEstimatedPose(PoseOrigin.ofGlobal());
        if (frontPosOpt.isPresent()) {
            drivetrain.getOdometryThread().addVisionPoseEstimate(frontPosOpt.get());
        }

        
        Optional<BreakerEstimatedPose> backPosOpt = backPosSrc.getEstimatedPose(PoseOrigin.ofGlobal());
        if (backPosOpt.isPresent()) {
            drivetrain.getOdometryThread().addVisionPoseEstimate(backPosOpt.get());
        }
        
        // Optional<BreakerEstimatedPose> estPosOpt = backPosSrc.getEstimatedPose(PoseOrigin.ofGlobal());
        // if (estPosOpt.isPresent()) {
        //     BreakerEstimatedPose estPos = estPosOpt.get();
        //     BreakerLog.recordOutput("pose", Pose2d.struct, estPos.estimatedPose.toPose2d());
        //     Matrix<N3, N1> devMatrix = estPos.estimationStandardDevations.get();
        //     BreakerLog.recordOutput("StdDevs", new double[]{devMatrix.get(0, 0), devMatrix.get(1, 0), devMatrix.get(2, 0)});
        // }
        
    }
}
