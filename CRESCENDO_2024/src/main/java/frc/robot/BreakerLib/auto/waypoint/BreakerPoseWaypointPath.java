// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerMath.MirrorSymetryAxis2d;

/** Add your docs here. */
public class BreakerPoseWaypointPath {
    private final double maxVelocity;
    private final Pose2d[] waypoints;

    public BreakerPoseWaypointPath(double maxVelocity, Pose2d... waypoints) {
        this.maxVelocity = maxVelocity;
        this.waypoints = waypoints;
    }

    /** @return Total distance between all waypoints in meters. */
    public double getTotalPathDistance() {
        double dist = 0;
        for (int i = 1; i < waypoints.length; i++) {
            dist += waypoints[i - 1].getTranslation().getDistance(waypoints[i].getTranslation());
        }
        return dist;
    }

    /** @return Max vel constraint of path. */
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /** @return Array of 2d waypoints. */
    public Pose2d[] getWaypoints() {
        return waypoints;
    }

    /**
     * @return New waypoint path with all points from both paths and mean trapezoid
     *         profile constraints.
     * 
     * @param other Other waypoint path.
     */
    public BreakerPoseWaypointPath concatenate(BreakerPoseWaypointPath other) {
        Pose2d[] newWaypoints = new Pose2d[getWaypoints().length + other.getWaypoints().length];
        for (int i = 0; i < newWaypoints.length; i++) {
            if (i < waypoints.length) {
                newWaypoints[i] = waypoints[i];
            } else {
                newWaypoints[i] = other.getWaypoints()[i - (newWaypoints.length - 1)];
            }
        }
        return new BreakerPoseWaypointPath(
                (maxVelocity + other.maxVelocity) / 2.0,
                newWaypoints);
    }

    public BreakerPoseWaypointPath mirror(double axisOfSymmetry) {
        Pose2d[] result = new Pose2d[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            result[i] = BreakerMath.mirrorPose(waypoints[i], axisOfSymmetry, MirrorSymetryAxis2d.Y, MirrorSymetryAxis2d.Y);
        }
        return new BreakerPoseWaypointPath(maxVelocity, result);
    }

}
