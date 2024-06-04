// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.Constants.VisionConstants.BACK_LEFT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.BACK_LEFT_CAMERA_TRANS;
import static frc.robot.Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.BACK_RIGHT_CAMERA_TRANS;
import static frc.robot.Constants.VisionConstants.FRONT_LEFT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.FRONT_LEFT_CAMERA_TRANS;
import static frc.robot.Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.FRONT_RIGHT_CAMERA_TRANS;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_NAME;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_TRANS;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimPhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.proto.Translation2dProto;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.devices.vision.limelight.BreakerLimelight;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera.BreakerPhotonVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerPoseEstimationStandardDeviationCalculator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

import org.ejml.simple.SimpleMatrix;
import org.opencv.osgi.OpenCVInterface;

/** Add your docs here. */
public class Vision extends SubsystemBase {
    public BreakerLimelight limelight;
    public BreakerPhotonCamera backRightCam, backLeftCam, frontRightCam, frontLeftCam;
    private BreakerPhotonVisionPoseEstimator backRightPosSrc, backLeftPosSrc, frontRightPosSrc, frontLeftPosSrc;
    private Drive drivetrain;
    private boolean odometryHasBeenSeededCashed;
    private BreakerPhotonVisionPoseEstimator[] poseSources;
    private ArrayList<BreakerEstimatedPose> estimatedPoses;
    private boolean enable;
    private Optional<NoteTarget> trackedNote;

    public Vision(Drive drivetrain, boolean enable) {
        limelight = new BreakerLimelight(LIMELIGHT_NAME, LIMELIGHT_TRANS);

        backRightCam = new BreakerPhotonCamera(BACK_RIGHT_CAMERA_NAME, BACK_RIGHT_CAMERA_TRANS);
        backLeftCam = new BreakerPhotonCamera(BACK_LEFT_CAMERA_NAME, BACK_LEFT_CAMERA_TRANS);
        frontRightCam = new BreakerPhotonCamera(FRONT_RIGHT_CAMERA_NAME, FRONT_RIGHT_CAMERA_TRANS);
        frontLeftCam = new BreakerPhotonCamera(FRONT_LEFT_CAMERA_NAME, FRONT_LEFT_CAMERA_TRANS);

        backRightPosSrc = backRightCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        backLeftPosSrc = backLeftCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        frontRightPosSrc = frontRightCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        frontLeftPosSrc = frontLeftCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        poseSources = new BreakerPhotonVisionPoseEstimator[]{backLeftPosSrc, backRightPosSrc, frontRightPosSrc, frontLeftPosSrc};
        estimatedPoses = new ArrayList<>();
        this.drivetrain = drivetrain;
        odometryHasBeenSeededCashed = false;
        this.enable = enable;
    }

    @Override
    public void periodic() {
        estimateRobotPose();
    }

    private void estimateRobotToBestNoteTranslation() {
        if (LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            LimelightHelpers.getLimelightNTDouble(LIMELIGHT_NAME, "thor");
        }
    }

    private void estimateRobotPose() {
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
                    BreakerEstimatedPose pos = posOpt.get();
                    List<PhotonTrackedTarget> targets = est.getTrackedTargets();
                    if (targets.size() == 1) {
                        PhotonTrackedTarget tgt = targets.get(0);
                        if (tgt.getPoseAmbiguity() >= 0.15) {
                            continue;
                        }
                        
                        final Pose3d actual = pos.estimatedPose;
                        final double fieldBorderMargin = 0.5;
                        final double zMargin = 0.75;

                        if (actual.getX() < -fieldBorderMargin
                            || actual.getX() > Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() + fieldBorderMargin
                            || actual.getY() < -fieldBorderMargin
                            || actual.getY() > Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() + fieldBorderMargin
                            || actual.getZ() < -zMargin
                            || actual.getZ() > zMargin) {
                                continue;
                        }
                    }                    
                    estimatedPoses.add(pos);
                }
            }

            sortByStandardDeviation(estimatedPoses);

            for (BreakerEstimatedPose estPos: estimatedPoses) {
                drivetrain.getOdometryThread().addVisionPoseEstimate(estPos);
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

    public static record NoteTarget() {
        
    }
    
}
