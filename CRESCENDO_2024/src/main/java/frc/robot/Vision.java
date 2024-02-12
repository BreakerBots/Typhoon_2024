// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.BreakerLib.devices.vision.limelight.BreakerLimelight;
import frc.robot.BreakerLib.devices.vision.photon.BreakerPhotonCamera;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPoseSource;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerPoseEstimationStandardDevationCalculator;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.subsystems.Drive;
import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Vision extends SubsystemBase {
    public BreakerLimelight limelight;
    public BreakerPhotonCamera frontCam, backCam;
    private BreakerEstimatedPoseSource backPosSrc;

    public Vision(Drive drivetrain) {
        limelight = new BreakerLimelight(LIMELIGHT_NAME, LIMELIGHT_TRANS);
        frontCam = new BreakerPhotonCamera(FRONT_CAMERA_NAME, FRONT_CAMERA_TRANS);
        backCam = new BreakerPhotonCamera(BACK_CAMERA_NAME, BACK_CAMERA_TRANS);
        backPosSrc = backCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDevationCalculator());
        //drivetrain.getOdometryThread().registerEstimatedPoseSources(frontCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDevationCalculator()), backCam.getEstimatedPoseSource(APRIL_TAG_FIELD_LAYOUT, new BreakerPoseEstimationStandardDevationCalculator()));
    }

    @Override
    public void periodic() {
        Optional<BreakerEstimatedPose> estPosOpt = backPosSrc.getEstimatedPose(PoseOrigin.ofGlobal());
        if (estPosOpt.isPresent()) {
            BreakerEstimatedPose estPos = estPosOpt.get();
            BreakerLog.recordOutput("pose", Pose2d.struct, estPos.estimatedPose.toPose2d());
            Matrix<N3, N1> devMatrix = estPos.estimationStandardDevations.get();
            BreakerLog.recordOutput("StdDevs", new double[]{devMatrix.get(0, 0), devMatrix.get(1, 0), devMatrix.get(2, 0)});
        }
        
    }
}
