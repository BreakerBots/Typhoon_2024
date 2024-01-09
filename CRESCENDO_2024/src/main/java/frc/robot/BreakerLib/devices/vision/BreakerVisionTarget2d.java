// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision;

import org.opencv.photo.Photo;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class BreakerVisionTarget2d {
    private Transform3d robotToCameraTransform;
    private double yawDegrees;
    private double pitchDegrees; 
    private double area;
    private double captureTimestamp;
    public BreakerVisionTarget2d(Transform3d robotToCameraTransform, double yawDegrees, double pitchDegrees, double area, double captureTimestamp) {
        
    }

    public double getTargetToCameraDistance(double targetCenterHeightMeters) {
        return PhotonUtils.calculateDistanceToTargetMeters(robotToCameraTransform.getZ(), targetCenterHeightMeters, robotToCameraTransform.getRotation().getY(), Math.toRadians(pitchDegrees));
    }

    public Translation3d getCameraToTargetTranslation3d(double targetCenterHeightMeters) {
        Translation2d trans = PhotonUtils.estimateCameraToTargetTranslation(getTargetToCameraDistance(targetCenterHeightMeters), Rotation2d.fromDegrees(yawDegrees));
        return new Translation3d(trans.getX(), trans.getY(), targetCenterHeightMeters);
    }

    public Translation2d getCameraToTargetTranslation(double targetCenterHeightMeters) {
        return PhotonUtils.estimateCameraToTargetTranslation(getTargetToCameraDistance(targetCenterHeightMeters), Rotation2d.fromDegrees(yawDegrees));
    }

    public double getTargetToRobotDistance(double targetCenterHeightMeters) {
        return getCameraToTargetTranslation(targetCenterHeightMeters).plus(robotToCameraTransform.getTranslation().toTranslation2d()).getNorm();
    }

    public double getArea() {
        return area;
    }

    public double getPitchDegrees() {
        return pitchDegrees;
    }

    public Transform3d getRobotToCameraTransform() {
        return robotToCameraTransform;
    }

    public double getYawDegrees() {
        return yawDegrees;
    }

    public double getCaptureTimestamp() {
        return captureTimestamp;
    }

    

}
