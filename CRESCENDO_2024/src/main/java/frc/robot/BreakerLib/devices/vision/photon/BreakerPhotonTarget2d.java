// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photon;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/**
 * 2d Photon camera target. Ideally used to target game pieces, etc. on the
 * field.
 */
public class BreakerPhotonTarget2d extends SubsystemBase {

    private BreakerPhotonCamera camera;
    private BreakerGenericOdometer odometryProvider;

    private double targetHeightMeters;
    private PhotonTrackedTarget assignedTarget;
    private Supplier<PhotonTrackedTarget> assignedTargetSupplier;
    
    private boolean assignedTargetFound;
    private double targetFoundTimestamp;

    /**
     * Creates a new BreakerPhotonTarget with a given predefined target.
     * 
     * @param camera                 Photon camera.
     * @param odometryProvider       Odometer for field distance calculations.
     * @param assignedTargetSupplier Supplies {@link PhotonTrackedTarget} to look
     *                               for.
     * @param targetHeightMeters     Target height from ground.
     */
    public BreakerPhotonTarget2d(BreakerPhotonCamera camera, BreakerGenericOdometer odometryProvider, Supplier<PhotonTrackedTarget> assignedTargetSupplier,
            double targetHeightMeters) {
        this.camera = camera;
        this.odometryProvider = odometryProvider;
        this.assignedTargetSupplier = assignedTargetSupplier;
        assignedTarget = assignedTargetSupplier.get();
        assignedTargetFound = Objects.isNull(assignedTarget);
        this.targetHeightMeters = targetHeightMeters;
    }

    /** Logic used to find a target. */
    private void findAssignedTarget() {
        if (camera.hasTargets()) {
            assignedTarget = assignedTargetSupplier.get();
            assignedTargetFound = Objects.isNull(assignedTarget);
            if (assignedTargetFound) {
                targetFoundTimestamp = Timer.getFPGATimestamp();
            }
        }
    }

    /** @return Overall distance from target to camera. */
    public double getTargetDistanceMeters() {
        return PhotonUtils.calculateDistanceToTargetMeters(BreakerUnits.inchesToMeters(camera.getCameraHeight()),
                targetHeightMeters, Math.toRadians(camera.getCameraPitch()),
                Math.toRadians(getPitch()));
    }

    /** @return The relative distances in X and Y between the target and camera */
    public Translation2d getTargetTranslationFromCamera() {
        return PhotonUtils.estimateCameraToTargetTranslation(getTargetDistanceMeters(),
                Rotation2d.fromDegrees(getYaw()));
    }

    /**
     * @return The calculated X and Y coordinates of the target relative to the
     *         field
     *         based on vision and odometry.
     */
    public Translation2d getTargetTranslationFromField() {
        return odometryProvider.getOdometryPoseMeters().getTranslation().plus(getTargetTranslationFromCamera());
    }

    /** @return Assigned target yaw. */
    public double getYaw() {
        return assignedTarget.getYaw();
    }

    /** @return Assigned target pitch. */
    public double getPitch() {
        return assignedTarget.getPitch();
    }

    /** @return Assigned target skew. */
    public double getSkew() {
        return assignedTarget.getSkew();
    }

    /** @return Assigned target area. */
    public double getArea() {
        return assignedTarget.getArea();
    }

    /** List of target corner coordinates. */
    public List<TargetCorner> getTargetCorners() {
        return assignedTarget.getDetectedCorners();
    }

    /** @return True if assigned target is found. */
    public boolean getAssignedTargetFound() {
        return assignedTargetFound;
    }

    /** @return Latency of target data in seconds. */
    public double getTargetDataAge() {
        double timediffsec = Timer.getFPGATimestamp() - targetFoundTimestamp;
        return Units.millisecondsToSeconds(camera.getPipelineLatancyMilliseconds()) + timediffsec;
    }

    @Override
    public void periodic() {
        findAssignedTarget();
    }
}
