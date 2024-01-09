// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint.pathfinder;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerlegacyPathfinderNode.BreakerPathfinderNodeHeuristics;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerlegacyPathfinderNode.BreakerPathfinderNodeInstance;
import frc.robot.BreakerLib.auto.waypoint.pathfinder.BreakerLegacyPathfinderNodeGrid.BreakerPathfinderNodeGridInstance;

/** Add your docs here. */
public class BreakerLegacyPathfinder {
    private BreakerPathfinderNodeInstance curNode, startNode, endNode;
    private BreakerPathfinderNodeInstance[][] nodes;
    private ArrayList<BreakerPathfinderNodeInstance> openNodeList;
    private ArrayList<BreakerPathfinderNodeInstance> checkedNodeList;
    private boolean goalReached = false;
    private double searchTimeoutSeconds, nodeSideLengthMeters;

    public BreakerLegacyPathfinder(double searchTimeoutSeconds, BreakerPathfinderNodeGridInstance gridInstance) {
        this.searchTimeoutSeconds = searchTimeoutSeconds;
        nodes = gridInstance.getInstanceNodeGridArray();
        openNodeList = new ArrayList<>();
        checkedNodeList = new ArrayList<>();
        curNode = gridInstance.getStartNodeInstance();
        endNode = gridInstance.getEndNodeInstance();
        nodeSideLengthMeters = gridInstance.getNodeSideLengthMeters();
    }

    public BreakerLegacyPathfinderPath calculatePath() throws PathCalculationTimeOverrunException {
        Timer timer = new Timer();
        timer.start();
        while (!goalReached && timer.get() < searchTimeoutSeconds){
            int col = curNode.getGridPosX();
            int row = curNode.getGridPosY();

            curNode.setChecked(true);
            checkedNodeList.add(curNode);
            openNodeList.remove(curNode);

            if (row+1 < nodes[col].length) {
            //up
            openNode(nodes[col][row+1]);
            }

            if (row-1 >= 0) {
            //down
            openNode(nodes[col][row-1]);
            }

            if (col+1 < nodes.length) {
            //right
            openNode(nodes[col-1][row]);
            }

            if (col-1 >= 0) {
            //left
            openNode(nodes[col-1][row]);
            }

            int bestNodeIndex = 0;
            int bestNodeCostF = Integer.MAX_VALUE;

            for (int i = 0; i < openNodeList.size(); i++) {
                BreakerPathfinderNodeHeuristics nodeHeuristics = openNodeList.get(i).getHeuristics();
                if (nodeHeuristics.getF() < bestNodeCostF) {
                    bestNodeIndex = i;
                    bestNodeCostF = nodeHeuristics.getF();
                } else if (nodeHeuristics.getF() == bestNodeCostF && nodeHeuristics.getG() < openNodeList.get(i).getHeuristics().getG()) {
                    bestNodeIndex = i;
                }
            }

            curNode = openNodeList.get(bestNodeIndex);
            if (curNode.equals(endNode)) {
                goalReached = true;
            }
        }  

        if (goalReached) {
            return new BreakerLegacyPathfinderPath(nodeSideLengthMeters, trackPath());
        }
        throw new PathCalculationTimeOverrunException();
    }

    private void openNode(BreakerPathfinderNodeInstance node) {
        if (!node.isOpen() && !node.isChecked() && !node.isBarrier()) {
            node.setOpen(true);
            node.setParent(curNode);
            openNodeList.add(node);
        }
    }

    private ArrayList<BreakerlegacyPathfinderNode> trackPath() {
        BreakerPathfinderNodeInstance cur = endNode;
        ArrayList<BreakerlegacyPathfinderNode> nodePath = new ArrayList<>();
        nodePath.add(endNode.getBaseNode());
        while (!cur.equals(startNode)) {
            cur = cur.getParent();
            if (!cur.equals(startNode)) {
                nodePath.add(0, cur.getBaseNode());
            }
        }
        nodePath.add(0, startNode.getBaseNode());
        return nodePath;
    }

    public class PathCalculationTimeOverrunException extends Exception {
        public PathCalculationTimeOverrunException() {
            super("Your dynamic pathfinder path failed to calculate within the alloted time constraint");
        }
    }


    
}
