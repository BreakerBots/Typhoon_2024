// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.limelight.BreakerLimelight;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera.BreakerPhotonVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerPoseEstimationStandardDeviationCalculator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.subsystems.Drive;

/** Add your docs here. */
public class Vision extends SubsystemBase {
    public BreakerLimelight limelight;
    public BreakerPhotonCamera frontCam, leftCam, rightCam, backCam;
    private BreakerPhotonVisionPoseEstimator frontPosSrc, leftPosSrc, rightPosSrc, backPosSrc;
    private Drive drivetrain;
    private boolean odometryHasBeenSeededCashed;
    private BreakerPhotonVisionPoseEstimator[] poseSources;
    private ArrayList<BreakerEstimatedPose> estimatedPoses;
    private boolean enable;

    public Vision(Drive drivetrain, boolean enable) {
        limelight = new BreakerLimelight(LIMELIGHT_NAME, LIMELIGHT_TRANS);
        frontCam = new BreakerPhotonCamera(FRONT_CAMERA_NAME, FRONT_CAMERA_TRANS);
        leftCam = new BreakerPhotonCamera(LEFT_CAMERA_NAME, LEFT_CAMERA_TRANS);
        rightCam = new BreakerPhotonCamera(RIGHT_CAMERA_NAME, RIGHT_CAMERA_TRANS);
        backCam = new BreakerPhotonCamera(BACK_CAMERA_NAME, BACK_CAMERA_TRANS);

        frontPosSrc = frontCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        leftPosSrc = leftCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        rightPosSrc = rightCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        backPosSrc = backCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        poseSources = new BreakerPhotonVisionPoseEstimator[]{frontPosSrc, backPosSrc, leftPosSrc, rightPosSrc};
        estimatedPoses = new ArrayList<>();
        this.drivetrain = drivetrain;
        odometryHasBeenSeededCashed = false;
        this.enable = enable;
    }

    @Override
    public void periodic() {




        if (enable) {
            Pose2d odometryRefPos = drivetrain.getOdometryPoseMeters();
            estimatedPoses.clear();


            if (!odometryHasBeenSeededCashed) {
                if (drivetrain.getOdometryThread().hasBeenVisionSeeded()) {
                    odometryHasBeenSeededCashed = true;
                    for (BreakerPhotonVisionPoseEstimator est: poseSources) {
                        est.getPoseEstimator().setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
                    }
                } else {
                    odometryHasBeenSeededCashed = false;
                }
            }
            

            for (BreakerPhotonVisionPoseEstimator est: poseSources) {
                if (odometryHasBeenSeededCashed) {
                    est.getPoseEstimator().setReferencePose(odometryRefPos);
                } 

                Optional<BreakerEstimatedPose> posOpt = est.getEstimatedPose(PoseOrigin.ofGlobal());
                if (posOpt.isPresent()) {
                    estimatedPoses.add(posOpt.get());
                   
                }
            }

            sortByStandardDeviation(estimatedPoses);

            for (BreakerEstimatedPose estPos: estimatedPoses) {
                drivetrain.getOdometryThread().addVisionPoseEstimate(estPos);
            }
        }
    }

    private void sortByStandardDeviation(ArrayList<BreakerEstimatedPose> poses) {
        int i, j;
        BreakerEstimatedPose temp;
        boolean swapped;

        for (i = 0; i < poses.size() - 1; i++) {
            swapped = false;
            for (j = 0; j < poses.size() - i - 1; j++) {
                BreakerEstimatedPose jEstpos = poses.get(j);
                BreakerEstimatedPose j1Estpos = poses.get(j);

                if (jEstpos.estimationStandardDevations.isPresent() && j1Estpos.estimationStandardDevations.isPresent()) {
                    if (jEstpos.estimationStandardDevations.get().get(2, 0) < j1Estpos.estimationStandardDevations.get().get(2, 0)) {
                        temp = poses.get(j);
                        poses.set(poses.indexOf(jEstpos), j1Estpos);
                        poses.set(poses.indexOf(j1Estpos), temp);
                        swapped = true;
                    }
                }
                // Nones get moved to front of list
                else if (jEstpos.estimationStandardDevations.isPresent() && !j1Estpos.estimationStandardDevations.isPresent()) {
                    temp = poses.get(j);
                    poses.set(poses.indexOf(jEstpos), j1Estpos);
                    poses.set(poses.indexOf(j1Estpos), temp);
                    swapped = true;
                }
            }
 
            if (swapped == false)
                break;
        }
    }
}
