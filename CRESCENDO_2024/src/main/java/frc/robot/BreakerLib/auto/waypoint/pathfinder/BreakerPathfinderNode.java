// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint.pathfinder;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class BreakerPathfinderNode {
    private String nodeName;
    private Translation2d nodeLocation;
    private ArrayList<BreakerPathfinderNode> linkedNodes;
    public BreakerPathfinderNode(String nodeName, Translation2d nodeLocation) {
        this.nodeName = nodeName;
        this.nodeLocation = nodeLocation;
        linkedNodes = new ArrayList<>();
    }

    public void addLink(BreakerPathfinderNode node) {
        linkedNodes.add(node);
    }

    public Translation2d getLocation() {
        return nodeLocation;
    }

    public String getName() {
        return nodeName;
    }

    @Override
    public boolean equals(Object obj) {
        BreakerPathfinderNode node = (BreakerPathfinderNode) obj;
        return node.nodeLocation.equals(nodeLocation) && node.linkedNodes.equals(linkedNodes);
    }
}
