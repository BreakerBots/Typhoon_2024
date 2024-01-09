// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint.pathfinder;

/** Add your docs here. */
public class BreakerPathfinderUtil {
    public static void linkNodes(BreakerPathfinderNode node0, BreakerPathfinderNode node1) {
        node0.addLink(node1);
        node1.addLink(node0);
    }

    public static double getLinkLength(BreakerPathfinderNode node0, BreakerPathfinderNode node1) {
        return node0.getLocation().getDistance(node1.getLocation());
    }
}
