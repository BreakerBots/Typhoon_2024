// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photon;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPoseSource;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseCordinateSystem;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightResults;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightTarget_Fiducial;

/** Photon camera */
public class BreakerPhotonCamera extends BreakerGenericDevice implements BreakerEstimatedPoseSourceProvider {

    private PhotonCamera camera;
    private final String cameraName;
    private double cameraMountPitch; // Mounting angle
    private double cameraHeightMeters; // Height relative to ground.
    private Transform3d cameraPositionRelativeToRobot; // Height relative to ground, all else relative to robot position.

    /**
     * Creates a new camera that uses a PhotonVision-based computer vision
     * algorithem
     * 
     * @param cameraName                    Name of camera used to retreive data.
     * @param cameraPositionRelativeToRobot Transformation between robot center and
     *                                      camera position (Z translation is
     *                                      relative to the ground).
     */
    public BreakerPhotonCamera(String cameraName, Transform3d cameraPositionRelativeToRobot) {
        camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        deviceName = cameraName;
        this.cameraPositionRelativeToRobot = cameraPositionRelativeToRobot;
        this.cameraMountPitch = cameraPositionRelativeToRobot.getRotation().getY();
        this.cameraHeightMeters = cameraPositionRelativeToRobot.getZ();
    }

    public PhotonCamera getBaseCamera() {
        return camera;
    }

    // /** @return Overall raw result from photon camera. */
    // public PhotonPipelineResult getLatestRawResult() {
    //     return camera.getLatestResult();
    // }

    // /** @return If camera is locked onto any targets. */
    // public boolean hasTargets() {
    //     return getLatestRawResult().hasTargets();
    // }

    // /**
    //  * @return List of all raw PhotonTrackedTargets the camera has in its field of
    //  * view.
    //  */
    // public PhotonTrackedTarget[] getAllRawTrackedTargets() {
    //     return getLatestRawResult().targets.toArray(new PhotonTrackedTarget[getLatestRawResult().targets.size()]);
    // }

    // /** @return Number of camera targets currently locked on. */
    // public int getNumberOfCameraTargets() {
    //     return getAllRawTrackedTargets().length;
    // }

    // /** @return Camera latency in milliseconds */
    // public double getPipelineLatancyMilliseconds() {
    //     return getLatestRawResult().getLatencyMillis();
    // }

    /** Sets camera pipeline based on given number. */
    public void setPipelineIndex(int pipeNum) {
        camera.setPipelineIndex(pipeNum);
    }

    /** @return Index of the active pipeline on the camera as an integer. */
    public int getCurrentPipelineIndex() {
        return camera.getPipelineIndex();
    }

    /**
    //  * @return The raw PhotonTrackedTarget object representing the best tracked
    //  * target according to the pipeline's native sort
    //  */
    // public PhotonTrackedTarget getBestTarget() {
    //     return getLatestRawResult().getBestTarget();
    // }

    /** @return Height relative to ground in meters. */
    public double getCameraHeight() {
        return cameraHeightMeters;
    }

    /** @return pitch of camera in degrees. */
    public double getCameraPitch() {
        return cameraMountPitch;
    }

    /** @return 2d pose of camera relative to robot. */
    public Transform2d getCamPositionRelativeToRobot() {
        return new Transform2d(cameraPositionRelativeToRobot.getTranslation().toTranslation2d(), cameraPositionRelativeToRobot.getRotation().toRotation2d());
    }

    /** @return 3d pose of camera relative to robot. */
    public Transform3d get3dCamPositionRelativeToRobot() {
        return cameraPositionRelativeToRobot;
    }

    /** Change camera position relative to robot. */
    public void updateCamPositionRelativeToRobot(Transform3d newTransform) {
        cameraPositionRelativeToRobot = newTransform;
        cameraMountPitch = Math.toDegrees(newTransform.getRotation().getY());
        cameraHeightMeters = newTransform.getTranslation().getZ();
    }

    /** Turn on or tunn off LEDs. */
    public void setLEDMode(VisionLEDMode ledMode) {
        camera.setLED(ledMode);
    }

    /** @return Current state of camera LEDs. */
    public VisionLEDMode getCurrentLEDMode() {
        return camera.getLEDMode();
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        health = DeviceHealth.NOMINAL;
        // if (getPipelineLatancyMilliseconds() == 0) {
        //     health = DeviceHealth.INOPERABLE;
        //     faultStr = " camera_not_connected ";
        // }
    }

    
    /** 
     * @return String
     */
    @Override
    public String getDeviceName() {
        return cameraName;
    }

    
    /** 
     * @param isUnderAutomaticControl(
     */
    @Override
    // DOES NOUTHING, exists to satisfy BreakerGenericDevice Interface
    public void setDeviceName(String newName) {}



    public class BreakerPhotonVisionPoseEstimator implements BreakerEstimatedPoseSource {
        private AprilTagFieldLayout apriltagFieldLayout;
        private Optional<Function<BreakerEstimatedPose, Matrix<N3, N1>>> stdDevCalculation;
        private PhotonPoseEstimator poseEstimator;
        private List<PhotonTrackedTarget> trackedTargets;
        private BreakerPhotonVisionPoseEstimator(AprilTagFieldLayout apriltagFieldLayout, boolean pnpOnCoprocessor, Optional<Function<BreakerEstimatedPose, Matrix<N3, N1>>> stdDevCalculation) {
            this.apriltagFieldLayout = apriltagFieldLayout;
            this.stdDevCalculation = stdDevCalculation;
            PoseStrategy poseStrategy = (pnpOnCoprocessor ? PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR : PoseStrategy.MULTI_TAG_PNP_ON_RIO);
            poseEstimator = new PhotonPoseEstimator(apriltagFieldLayout, poseStrategy, camera, cameraPositionRelativeToRobot);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            trackedTargets = new ArrayList<PhotonTrackedTarget>();
        }

        public PhotonPoseEstimator getPoseEstimator() {
            return poseEstimator;
        }

        public List<PhotonTrackedTarget> getTrackedTargets() {
            return trackedTargets;
        }

        public Optional<BreakerEstimatedPose> getEstimatedPose(PoseOrigin origin) {
            Optional<EstimatedRobotPose> pvEstPoseOpt = poseEstimator.update();
            Optional<PoseCordinateSystem> chordSys = Optional.empty();
            trackedTargets.clear();
            if (pvEstPoseOpt.isPresent()) {
               chordSys = origin.getCordinateSystem();
                trackedTargets = pvEstPoseOpt.get().targetsUsed;
            } else {
                return Optional.empty();
            }
           
            if (chordSys.isPresent()) {
               Pose3d pos = chordSys.get().fromGlobal(pvEstPoseOpt.get().estimatedPose);
               BreakerEstimatedPose estPose = new BreakerEstimatedPose(pos,pvEstPoseOpt.get().timestampSeconds, chordSys.get(), pvFiducialTargetsToApriltagArray(pvEstPoseOpt.get().targetsUsed));
               if (stdDevCalculation.isPresent()) {
                    estPose = new BreakerEstimatedPose(estPose, stdDevCalculation.get().apply(estPose));
               }
               BreakerLog.recordOutput(cameraName + " POSE", Pose2d.struct, estPose.estimatedPose.toPose2d());
               return Optional.of(estPose);
            } else {
                return Optional.empty();
            }
        }

        private AprilTag[] pvFiducialTargetsToApriltagArray(List<PhotonTrackedTarget> fiducialTargets) {
            AprilTag[] atArr = new AprilTag[fiducialTargets.size()];
            for (int i = 0; i < fiducialTargets.size(); i++) {
                for (AprilTag at : apriltagFieldLayout.getTags()) {
                    if (at.ID == fiducialTargets.get(i).getFiducialId()) {
                        atArr[i] = at;
                        break;
                    }
                }
            }
            return atArr;
        }
    }



    @Override
    public BreakerPhotonVisionPoseEstimator getEstimatedPoseSource(AprilTagFieldLayout apriltagFieldLayout) {
        return new BreakerPhotonVisionPoseEstimator(apriltagFieldLayout, true, Optional.empty());
    }

    @Override
    public BreakerPhotonVisionPoseEstimator getEstimatedPoseSource(AprilTagFieldLayout apriltagFieldLayout,
            Function<BreakerEstimatedPose, Matrix<N3, N1>> stdDevCalculation) {
        return new BreakerPhotonVisionPoseEstimator(apriltagFieldLayout, true, Optional.of(stdDevCalculation));
    }

}
