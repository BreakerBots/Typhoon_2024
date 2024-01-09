// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint.pathfinder;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;

/** Add your docs here. */
public class BreakerLegacyPathfinderPath {
    private ArrayList<BreakerlegacyPathfinderNode> pathNodes;
    private double nodeSideLength;
    public BreakerLegacyPathfinderPath(double nodeSideLength, ArrayList<BreakerlegacyPathfinderNode> pathNodes) {
        this.pathNodes = new ArrayList<BreakerlegacyPathfinderNode>(pathNodes);
        this.nodeSideLength = nodeSideLength;
    }

    public ArrayList<BreakerlegacyPathfinderNode> getPathNodes() {
        return new ArrayList<BreakerlegacyPathfinderNode>(pathNodes);
    }

    public BreakerWaypointPath getAsWaypointPath(double maxVelocity) {
        Translation2d[] waypoints = new Translation2d[pathNodes.size()];
        for (int i = 0; i < waypoints.length; i++) {
            BreakerlegacyPathfinderNode node = pathNodes.get(i);
            double x = ((double)(node.getGridPosX()) + 0.5) * nodeSideLength;
            double y = ((double)(node.getGridPosY()) + 0.5) * nodeSideLength;

            waypoints[i] = new Translation2d(x, y);
        }
        return new BreakerWaypointPath(maxVelocity, waypoints);
    }

    public BreakerWaypointPath getAsWaypointPath(double maxVelocity, Translation2d startPoint, Translation2d endPoint) {
        Translation2d[] waypoints = new Translation2d[pathNodes.size()];
        for (int i = 1; i < waypoints.length - 1; i++) {
            BreakerlegacyPathfinderNode node = pathNodes.get(i);
            double x = ((double)(node.getGridPosX()) + 0.5) * nodeSideLength;
            double y = ((double)(node.getGridPosY()) + 0.5) * nodeSideLength;

            waypoints[i] = new Translation2d(x, y);
        }
        waypoints[0] = startPoint;
        waypoints[waypoints.length-1] = endPoint;
        return new BreakerWaypointPath(maxVelocity, waypoints);
    }
}
