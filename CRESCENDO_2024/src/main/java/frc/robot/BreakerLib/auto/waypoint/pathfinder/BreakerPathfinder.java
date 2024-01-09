// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint.pathfinder;

import java.util.ArrayList;

import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;

/** Add your docs here. */
public class BreakerPathfinder {
    private BreakerPathfinderNode[] nodes;
    public BreakerPathfinder(BreakerPathfinderNode... nodes) {
        this.nodes = nodes;
    }

    public ArrayList<BreakerPathfinderNode> findPath(BreakerPathfinderNode start, BreakerPathfinderNode goal) {
        boolean sFound = false;
        boolean gFound = false;

        ArrayList<ArrayList<BreakerPathfinderNode>> heldPaths = new ArrayList<>();
        ArrayList<ArrayList<BreakerPathfinderNode>> activePaths = new ArrayList<>();

        for (BreakerPathfinderNode n: nodes) {
            if (n.equals(start)) {
                sFound = true;
            }
            if (n.equals(goal)) {
                gFound = true;
            }
        }

        if (!(sFound && gFound)) {
            return new ArrayList<BreakerPathfinderNode>();
        }

        while (true) {

        }


        
    }
}
