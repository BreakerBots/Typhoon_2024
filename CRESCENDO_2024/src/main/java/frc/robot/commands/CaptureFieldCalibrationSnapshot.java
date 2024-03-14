// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.Vision.VisionCalibrationFrame;

public class CaptureFieldCalibrationSnapshot extends Command {
  private ArrayList<VisionCalibrationFrame> calibData;
  private ArrayList<AprilTag> knownTags;
  private AprilTagFieldLayout calibResults;
  private BreakerPhotonCamera[] cameras;
  private final Timer timer;
  private GenericEntry captureButton;
  /** Creates a new CalibrateVisionForFieldPnP. */
  public CaptureFieldCalibrationSnapshot() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (BreakerPhotonCamera cam: cameras) {
      PhotonPipelineResult res = cam.getBaseCamera().getLatestResult();
      List<PhotonTrackedTarget> tgts = res.targets;
      if (tgts.size() > 1) {
        calibData.add(new VisionCalibrationFrame(cam.get3dCamPositionRelativeToRobot(), cam.getBaseCamera().getCameraMatrix().get(), cam.getBaseCamera().getDistCoeffs().get(), res));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
