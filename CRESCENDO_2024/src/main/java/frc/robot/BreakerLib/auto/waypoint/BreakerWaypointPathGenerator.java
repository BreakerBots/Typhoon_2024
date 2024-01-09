// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerLegacyPathfinder;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerLegacyPathfinderNodeGrid;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerLegacyPathfinderPath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolatableDoubleArray;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerLegrangeInterpolateingTreeMap;

/** Generates waypoint paths for auto driving. */
public class BreakerWaypointPathGenerator {

    /**
     * 
     * 
     * @param maxVelocity
     * @param maxAcceleration
     * @param interpolationResolution
     * @param waypoints
     * @return
     */
    public static BreakerWaypointPath generateWaypointPath(double maxVelocity, double interpolationResolution, Translation2d... waypoints) {
        BreakerLegrangeInterpolateingTreeMap<Double, BreakerInterpolatableDoubleArray>  interMap = new BreakerLegrangeInterpolateingTreeMap<>();
        ArrayList<Translation2d> newWaypoints = new ArrayList<>();
        double dt = 1.0/interpolationResolution;
        for (int i = 0; i < waypoints.length; i++) {
            Translation2d wp = waypoints[i];
            interMap.put((double) i, new BreakerInterpolatableDoubleArray(new double[]{wp.getX(), wp.getY()}));
        }

        for (double i = 0; i < waypoints.length; i+=dt) {
            double[] interArr = interMap.getInterpolatedValue(i).getValue();
            newWaypoints.add(new Translation2d(interArr[0], interArr[1]));
        }

        return new BreakerWaypointPath(maxVelocity, newWaypoints.toArray(new Translation2d[newWaypoints.size()]));
    }

    public static BreakerWaypointPath findWaypointPath(double pathSearchTimeoutSeconds, double maxVelocity, Translation2d startPoint, Translation2d endPoint, BreakerLegacyPathfinderNodeGrid nodeGrid) throws Exception {
        try {
            BreakerLegacyPathfinder pathfinder = new BreakerLegacyPathfinder(pathSearchTimeoutSeconds, nodeGrid.getInstance(nodeGrid.getNodeFromPosition(startPoint), nodeGrid.getNodeFromPosition(startPoint)));
            BreakerLegacyPathfinderPath pfPath = pathfinder.calculatePath();
            return pfPath.getAsWaypointPath(maxVelocity, startPoint, endPoint);
        } catch (Exception e) {
            throw e;
        }
    }

    public static BreakerWaypointPath findWaypointPath(double pathSearchTimeoutSeconds, double maxVelocity, BreakerLegacyPathfinderNodeGrid nodeGrid, Translation2d... pathPoints) throws Exception {
        BreakerWaypointPath[] wpPaths = new BreakerWaypointPath[pathPoints.length - 1];
        double sTimeout = pathSearchTimeoutSeconds / (pathPoints.length - 1);
        for (int i = 0; i < pathPoints.length - 1; i++) {
            wpPaths[i] = findWaypointPath(sTimeout, maxVelocity, pathPoints[i], pathPoints[i+1], nodeGrid);
        }
        BreakerWaypointPath path = wpPaths[0];
        for (int i = 1; i < wpPaths.length; i++) {
            path.concatenate(wpPaths[i]);
        }
        return path;
    }
}
