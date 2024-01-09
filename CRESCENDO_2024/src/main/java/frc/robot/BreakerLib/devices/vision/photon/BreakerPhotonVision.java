// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photon;

import java.util.HashMap;
import java.util.Map.Entry;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.vision.posefilter.BreakerVisionPoseFilter;
import frc.robot.BreakerLib.position.odometry.vision.posefilter.BreakerVisionPoseFilterOdometer;

/** WIP */
public class BreakerPhotonVision implements BreakerGenericOdometer {
    private BreakerFiducialPhotonTarget[] targets;
    private BreakerPhotonCamera[] cameras;
    private BreakerVisionPoseFilter poseFilter;
    private BreakerVisionPoseFilterOdometer odometer;
    public BreakerPhotonVision(double poseFilterTrustCoef, double poseFilterMaxUncertanty, BreakerPhotonCamera[] cameras, AprilTagFieldLayout aprilTagFieldLayout) {
        targets = new BreakerFiducialPhotonTarget[aprilTagFieldLayout.getTags().size()];
        this.cameras = cameras;
        int i = 0;
        for (AprilTag at: aprilTagFieldLayout.getTags()) {
            targets[i] = new BreakerFiducialPhotonTarget(at, cameras);
            i++;
        }

        poseFilter = new BreakerVisionPoseFilter(poseFilterTrustCoef, poseFilterMaxUncertanty, targets);
        odometer = new BreakerVisionPoseFilterOdometer(poseFilter);
    }

    public BreakerPhotonVision(double poseFilterTrustCoef, double poseFilterMaxUncertanty, double distanceScailFactor, double maxDistance, BreakerPhotonCamera[] cameras, AprilTagFieldLayout aprilTagFieldLayout) {
        targets = new BreakerFiducialPhotonTarget[aprilTagFieldLayout.getTags().size()];
        this.cameras = cameras;
        int i = 0;
        for (AprilTag at: aprilTagFieldLayout.getTags()) {
            targets[i] = new BreakerFiducialPhotonTarget(at, cameras);
            i++;
        }

        poseFilter = new BreakerVisionPoseFilter(poseFilterTrustCoef, poseFilterMaxUncertanty, distanceScailFactor, maxDistance, targets);
        odometer = new BreakerVisionPoseFilterOdometer(poseFilter);
    }

    
    /** 
     * @return BreakerPhotonCamera[]
     */
    public BreakerPhotonCamera[] getCameras() {
        return cameras;
    }

    
    /** 
     * @param cameraName
     * @return BreakerPhotonCamera
     */
    public BreakerPhotonCamera getCamera(String cameraName) {
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.getDeviceName() == cameraName) {
                return cam;
            }
        }
        return null;
    }

    
    /** 
     * @return boolean
     */
    public boolean hasTargets() {
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.hasTargets()) {
                return true;
            }
        }
        return false;
    }

    
    /** 
     * @return BreakerFiducialPhotonTarget[]
     */
    public BreakerFiducialPhotonTarget[] getFiducialTargets() {
        return targets;
    }

    
    /** 
     * @return Pose3d
     */
    public Pose3d getFilteredRobotPose3d() {
        return poseFilter.getFilteredRobotPose3d();
    }

    
    /** 
     * @return Pose2d
     */
    public Pose2d getFilteredRobotPose() {
        return poseFilter.getFilteredRobotPose();
    }

    
    /** 
     * @return BreakerVisionOdometer
     */
    public BreakerVisionPoseFilterOdometer getBaseVisionOdometer() {
        return odometer;
    }

    
    /** 
     * @param newPose
     */
    @Override
    public void setOdometryPosition(Pose2d newPose) {
        odometer.setOdometryPosition(newPose);
    }

    
    /** 
     * @return Pose2d
     */
    @Override
    public Pose2d getOdometryPoseMeters() {
        return odometer.getOdometryPoseMeters();
    }

    
    /** 
     * @return BreakerMovementState2d
     */
    @Override
    public BreakerMovementState2d getMovementState() {
        return odometer.getMovementState();
    }

    
    /** 
     * @return ChassisSpeeds
     */
    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return odometer.getRobotRelativeChassisSpeeds();
    }

    
    /** 
     * @return ChassisSpeeds
     */
    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return odometer.getFieldRelativeChassisSpeeds();
    }
}
