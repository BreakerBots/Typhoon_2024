// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Deque;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OakD extends SubsystemBase {
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable datatable = inst.getTable("OAKD");
  private IntegerSubscriber numTargetsSub = datatable.getIntegerTopic("numTargets").subscribe(0);
  private int numTargets = 0;
  private ArrayList<PersistantSpatialTarget> persistantTargetStore = new ArrayList<>();
  private final double staleTargetForgetTime = 1.0;

  
  public OakD() {}

  public boolean hasTarget() {
    return numTargets > 0;
  }

  @Override
  public void periodic() {
    numTargets = (int) numTargetsSub.get();
    

  }

  public static class PersistantSpatialTarget {
    private ArrayList<SpatialDetection> detectionHistory;
    private int detectionHistoryMaxLength;
    private int objectID;

    public PersistantSpatialTarget(SpatialDetection initialDetection, int detectionHistoryMaxLength) {
      this.detectionHistoryMaxLength = Math.max(detectionHistoryMaxLength, 2);
      detectionHistory = new ArrayList<>();
      detectionHistory.add(initialDetection);
      objectID = initialDetection.getObjectID();
    }

    public void addDetection(SpatialDetection detection) {
      if (detection.getObjectID() != objectID) {
        throw new IllegalArgumentException(String.format("Object ID (%d) does not match (%d) of the PersistantSpatialTarget", detection.getObjectID(), objectID));
      }
      detectionHistory.add(0, detection);
      if (detectionHistory.size() > detectionHistoryMaxLength) {
        detectionHistory.remove(detectionHistory.size() - 1);
      }
    }

    

    
    
    
  }

  public static class SpatialDetection {
    private int objectID;


    public int getObjectID() {
      return objectID;
    }
  }

  public static class RawSpatial {
    public RawSpatial(Transform3d cameraTransform, double camspaceX, double camSpaceY, double camSpaceZ) {

    }

  }
}
