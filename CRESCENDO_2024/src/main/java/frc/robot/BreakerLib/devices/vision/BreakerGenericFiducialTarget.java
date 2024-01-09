// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision;

import java.util.List;
import java.util.Objects;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Fiducial marker target. Use if using AprilTags w/ 3D calibrated camera. */
public class BreakerFiducialTarget { 

    public double getTimestamp();

    public Transform3d getCameraToTargetTransform() {

    }

    public Transform3d getRobotToTargetTransform() {

    }

    /** @return ID of fiducial target */
    public int getFiducialID();
}
