// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.Constants.VisionConstants.BACK_LEFT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.BACK_LEFT_CAMERA_TRANS;
import static frc.robot.Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.BACK_RIGHT_CAMERA_TRANS;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_NAME;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_TRANS;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.limelight.BreakerLimelight;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera.BreakerPhotonVisionPoseEstimator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerPoseEstimationStandardDeviationCalculator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.subsystems.Drive;

/** Add your docs here. */
public class Vision extends SubsystemBase {
    public BreakerLimelight limelight;
    public BreakerPhotonCamera backRightCam, backLeftCam, rightCam, backCam;
    private BreakerPhotonVisionPoseEstimator backRightPosSrc, backLeftPosSrc, rightPosSrc, backPosSrc;
    private Drive drivetrain;
    private boolean odometryHasBeenSeededCashed;
    private BreakerPhotonVisionPoseEstimator[] poseSources;
    private ArrayList<BreakerEstimatedPose> estimatedPoses;
    private boolean enable;

    public Vision(Drive drivetrain, boolean enable) {
        limelight = new BreakerLimelight(LIMELIGHT_NAME, LIMELIGHT_TRANS);
        backRightCam = new BreakerPhotonCamera(BACK_RIGHT_CAMERA_NAME, BACK_RIGHT_CAMERA_TRANS);
        backLeftCam = new BreakerPhotonCamera(BACK_LEFT_CAMERA_NAME, BACK_LEFT_CAMERA_TRANS);
        // rightCam = new BreakerPhotonCamera(RIGHT_CAMERA_NAME, RIGHT_CAMERA_TRANS);
        // backCam = new BreakerPhotonCamera(BACK_CAMERA_NAME, BACK_CAMERA_TRANS);

        backRightPosSrc = backRightCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        backLeftPosSrc= backLeftCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        // rightPosSrc = rightCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        // backPosSrc = backCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDeviationCalculator());
        poseSources = new BreakerPhotonVisionPoseEstimator[]{backRightPosSrc, backLeftPosSrc};
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
                    BreakerEstimatedPose pos = posOpt.get();
                    List<PhotonTrackedTarget> targets = est.getTrackedTargets();
                    if (targets.size() == 1) {
                        PhotonTrackedTarget tgt = targets.get(0);
                        if (tgt.getPoseAmbiguity() <= 0.15) {
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

    // private AprilTagFieldLayout solveForFieldTagPositionsParalax(BreakerPhotonCamera left, BreakerPhotonCamera right, AprilTagFieldLayout idealLayout, ArrayList<VisionCalibrationFrame> calibrationFrames, int... knownGoodTagIDs) {
        
    //     ArrayList<BreakerTriplet<Double, FieldCalibrationCameraCapture, FieldCalibrationCameraCapture>> paralaxCaptures = new ArrayList<>();
    //     for (VisionCalibrationFrame frame:calibrationFrames) {
    //         Optional<FieldCalibrationCameraCapture> leftCapOpt = Optional.empty();
    //         Optional<FieldCalibrationCameraCapture> rightCapOpt = Optional.empty();
    //         for (FieldCalibrationCameraCapture capture :frame.cameraCaptures) {
    //             if (capture.camName.equals(left.getDeviceName())) {
    //                 leftCapOpt = Optional.of(capture);
    //             } else if (capture.camName.equals(right.getDeviceName())) {
    //                 rightCapOpt = Optional.of(capture);
    //             }
    //         }
    //         if (leftCapOpt.isPresent() && rightCapOpt.isPresent()) {
    //              FieldCalibrationCameraCapture leftCap = leftCapOpt.get();
    //              FieldCalibrationCameraCapture rightCap = rightCapOpt.get();
    //              double frameTimestamp = (leftCap.pipelineResult.getTimestampSeconds() + rightCap.pipelineResult.getTimestampSeconds()) / 2.0;
    //              paralaxCaptures.add(new BreakerTriplet<Double,FieldCalibrationCameraCapture, FieldCalibrationCameraCapture>(frameTimestamp, leftCap, rightCap));
    //         }
    //     }
    //     return null;
    // }
    
    // private static Optional<RobotToTargetStereoSolution> getRobotToTargetTransformSolution(AprilTag idealTag, Transform3d leftCamTransform, Transform3d rightCamTransform, ArrayList<BreakerTriplet<Double, FieldCalibrationCameraCapture, FieldCalibrationCameraCapture>> stereoCaptures) {
    //     BreakerInterpolatingTreeMap<Double, BreakerInterpolatableDoubleArray> leftAngles = new BreakerInterpolatingTreeMap<>();
    //     BreakerInterpolatingTreeMap<Double, BreakerInterpolatableDoubleArray> rightAngles = new BreakerInterpolatingTreeMap<>();
    //     for (BreakerTriplet<Double, FieldCalibrationCameraCapture, FieldCalibrationCameraCapture> stereoCap : stereoCaptures) {
    //         for (PhotonTrackedTarget tgt : stereoCap.getMiddle().pipelineResult.targets) {
    //             if (tgt.getFiducialId() == idealTag.ID) {
    //                 Rotation2d yaw = Rotation2d.fromDegrees(-tgt.getYaw());
    //                 Rotation2d pitch = Rotation2d.fromDegrees(tgt.getPitch());
    //                 leftAngles.put(stereoCap.getMiddle().pipelineResult.getTimestampSeconds(), new BreakerInterpolatableDoubleArray(yaw.getCos(), yaw.getSin(), pitch.getCos(), pitch.getSin()));
    //                 break;
    //             }
    //         }
    //         for (PhotonTrackedTarget tgt : stereoCap.getRight().pipelineResult.targets) {
    //             if (tgt.getFiducialId() == idealTag.ID) {
    //                 Rotation2d yaw = Rotation2d.fromDegrees(-tgt.getYaw());
    //                 Rotation2d pitch = Rotation2d.fromDegrees(tgt.getPitch());
    //                 leftAngles.put(stereoCap.getMiddle().pipelineResult.getTimestampSeconds(), new BreakerInterpolatableDoubleArray(yaw.getCos(), yaw.getSin(), pitch.getCos(), pitch.getSin()));
    //                 break;
    //             }
    //         }
    //     }
    //     if (leftAngles.size() != rightAngles.size() || leftAngles.size() < 50) {
    //         return Optional.empty();
    //     }
    //     return Optional.of(new RobotToTargetStereoSolution(idealTag, leftCamTransform, rightCamTransform, leftAngles, rightAngles));
    // }

    // public static class RobotToTargetStereoSolution {
    //     private AprilTag idealTag;
    //     private Transform3d leftCamTransform;
    //     private Transform3d rightCamTransform;
    //     private BreakerInterpolatingTreeMap<Double, BreakerInterpolatableDoubleArray> leftAngles;
    //     private BreakerInterpolatingTreeMap<Double, BreakerInterpolatableDoubleArray> rightAngles;
    //     private double baseDist;
    //     public RobotToTargetStereoSolution(AprilTag idealTag, Transform3d leftCamTransform, Transform3d rightCamTransform, BreakerInterpolatingTreeMap<Double, BreakerInterpolatableDoubleArray> leftAngles,  BreakerInterpolatingTreeMap<Double, BreakerInterpolatableDoubleArray> rightAngles) {

    //     }
    //     //              C
    //     //             /\
    //     //          a /  \ b
    //     //           /    \
    //     //         B ------ A
    //     //             c

    //     public Transform3d getTransformAtTime(double time) {

    //         BreakerInterpolatableDoubleArray leftAngArr = leftAngles.getInterpolatedValue(time);
    //         Rotation2d leftCameraYaw = new Rotation2d(leftAngArr.getValue()[0], leftAngArr.getValue()[1]).minus(Rotation2d.fromRadians(leftCamTransform.getRotation().getZ()));
    //         Rotation2d leftCameraPitch = new Rotation2d(leftAngArr.getValue()[2], leftAngArr.getValue()[3]).minus(Rotation2d.fromDegrees(leftCamTransform.getRotation().getY()));
    //         BreakerInterpolatableDoubleArray rightAngArr = rightAngles.getInterpolatedValue(time);
    //         Rotation2d rightCameraYaw = new Rotation2d(rightAngArr.getValue()[0], rightAngArr.getValue()[1]).minus(Rotation2d.fromRadians(rightCamTransform.getRotation().getZ()));
    //         Rotation2d rightCameraPitch = new Rotation2d(rightAngArr.getValue()[2], rightAngArr.getValue()[3]).minus(Rotation2d.fromRadians(rightCamTransform.getRotation().getY()));
            
            

    //     }
    // }
    public static record VisionCalibrationFrame(double frameTimestamp, ArrayList<FieldCalibrationCameraCapture> cameraCaptures) {};
    public static record FieldCalibrationCameraCapture(String camName, Transform3d robotToCamTransform, Matrix<N3, N3> cameraMatrix, Matrix<N5, N1> distCoeffs, PhotonPipelineResult pipelineResult) {};
}
