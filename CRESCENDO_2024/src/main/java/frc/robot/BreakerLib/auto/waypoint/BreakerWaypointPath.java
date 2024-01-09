// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A path of positional waypoints. Rotation must be affected separately with a
 * supplier
 */
public class BreakerWaypointPath {
    private final double maxVelocity;
    private final Translation2d[] waypoints;

    public BreakerWaypointPath(double maxVelocity, Translation2d... waypoints) {
        this.maxVelocity = maxVelocity;
        this.waypoints = waypoints;
    }

    /** @return Total distance between all waypoints in meters. */
    public double getTotalPathDistance() {
        double dist = 0;
        for (int i = 1; i < waypoints.length; i++) {
            dist += waypoints[i - 1].getDistance(waypoints[i]);
        }
        return dist;
    }

    /** @return Max vel constraint of path. */
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /** @return Array of 2d waypoints. */
    public Translation2d[] getWaypoints() {
        return waypoints;
    }

    /**
     * @return New waypoint path with all points from both paths and mean trapezoid profile constraints.
     * 
     * @param other Other waypoint path.
     */
    public BreakerWaypointPath concatenate(BreakerWaypointPath other) {
        Translation2d[] newWaypoints = new Translation2d[getWaypoints().length + other.getWaypoints().length];
        for (int i = 0; i < newWaypoints.length; i++) {
            if (i < waypoints.length) {
                newWaypoints[i] = waypoints[i];
            } else {
                newWaypoints[i] = other.getWaypoints()[i - (newWaypoints.length - 1)];
            }
        }
        return new BreakerWaypointPath(
            (maxVelocity + other.maxVelocity) / 2.0,
            newWaypoints);
    }

    public BreakerWaypointPath mirror(double axisOfSymmetry) {
        Translation2d[] result = new Translation2d [waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            double distance = axisOfSymmetry - waypoints[i].getX();
            result[i] = new Translation2d(axisOfSymmetry + distance, waypoints[i].getY());
        }
        return new BreakerWaypointPath(maxVelocity, result);
    }
}
