// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.limelight;

import java.util.Optional;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPose;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.BreakerEstimatedPoseSource;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightResults;
import frc.robot.BreakerLib.util.vendorutil.LimelightHelpers.LimelightTarget_Fiducial;

/** Add your docs here. */
public class BreakerLimelight extends BreakerGenericDevice implements BreakerEstimatedPoseSourceProvider {
    private final String limelightName;
    private final Transform3d cameraPositionRelativeToRobot;
    public BreakerLimelight(String limelightName, Transform3d cameraPositionRelativeToRobot) {
        this.limelightName = limelightName;
        this.cameraPositionRelativeToRobot = cameraPositionRelativeToRobot;
        LimelightHelpers.getCameraPose3d_RobotSpace(limelightName);
        getLatestResults();
    }

    public boolean hasTargets(BreakerLimelightTargetType targetType) {
        return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tv").getDouble(0) == 1;
    }

    public BreakerEstimatedPoseSource getEstimatedPoseSource(AprilTagFieldLayout apriltagFieldLayout) {
        return new BreakerLimelightPoseEstimator(apriltagFieldLayout, Optional.empty());
    }
    
    public BreakerEstimatedPoseSource getEstimatedPoseSource(AprilTagFieldLayout apriltagFieldLayout, Function<BreakerEstimatedPose, Matrix<N3, N1>> stdDevCalculation) {
        return new BreakerLimelightPoseEstimator(apriltagFieldLayout, Optional.of(stdDevCalculation));
    }

    public void setLEDMode(BreakerLimelightLEDMode ledMode) {
        switch(ledMode) {
            case BLINK:
                LimelightHelpers.setLEDMode_ForceBlink(limelightName);
                break;
            case OFF:
                LimelightHelpers.setLEDMode_ForceOff(limelightName);
                break;
            case ON:
                LimelightHelpers.setLEDMode_ForceOn(limelightName);
                break;
            case PIPELINE:
            default:
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
                break;
        }
    }

    public void setCameraMode(BreakerLimelightCameraMode cameraMode) {
        switch(cameraMode) {
            case DRIVER_CAMERA:
                break;
            case VISION_PROCESSOR:
                break;
            default:
                break;
            
        }
    }

    public class BreakerLimelightPoseEstimator implements BreakerEstimatedPoseSource {
        private AprilTagFieldLayout apriltagFieldLayout;
        private Optional<Function<BreakerEstimatedPose, Matrix<N3, N1>>> stdDevCalculation;
        private double lastUpdateTimestamp;
        private BreakerLimelightPoseEstimator(AprilTagFieldLayout apriltagFieldLayout, Optional<Function<BreakerEstimatedPose, Matrix<N3, N1>>> stdDevCalculation) {
            this.apriltagFieldLayout = apriltagFieldLayout;
            this.stdDevCalculation = stdDevCalculation;
            lastUpdateTimestamp = -1.0;
        }

        public Optional<BreakerEstimatedPose> getEstimatedPose(PoseOrigin origin) {
            LimelightResults results = getLatestResults();
            Optional<PoseCordinateSystem> chordSys = Optional.empty();
            Optional<Pose3d> estPoseObjOpt = Optional.empty();
            if (results.targetingResults.targets_Fiducials.length > 0 && results.targetingResults.valid && (Math.abs(results.targetingResults.timestamp_LIMELIGHT_publish - lastUpdateTimestamp) > 1e-5)) {
                MathUtil.isNear(lastUpdateTimestamp, lastUpdateTimestamp, lastUpdateTimestamp);
               chordSys = origin.getCordinateSystem();
               estPoseObjOpt = Optional.of(results.targetingResults.getBotPose3d_wpiBlue());
            } else {
                return Optional.empty();
            }
            if (chordSys.isPresent() && estPoseObjOpt.isPresent()) {
               Pose3d pos = chordSys.get().fromGlobal(estPoseObjOpt.get());
               BreakerEstimatedPose estPose = new BreakerEstimatedPose(pos, results.targetingResults.timestamp_LIMELIGHT_publish, chordSys.get(), limelightFiducialTargetsToApriltagArray(results.targetingResults.targets_Fiducials));
               if (stdDevCalculation.isPresent()) {
                    estPose = new BreakerEstimatedPose(estPose, stdDevCalculation.get().apply(estPose));
               }
               return Optional.of(estPose);
            } else {
                return Optional.empty();
            }
        }

        private AprilTag[] limelightFiducialTargetsToApriltagArray(LimelightTarget_Fiducial... fiducialTargets) {
            AprilTag[] atArr = new AprilTag[fiducialTargets.length];
            for (int i = 0; i < fiducialTargets.length; i++) {
                for (AprilTag at : apriltagFieldLayout.getTags()) {
                    if (at.ID == fiducialTargets[i].fiducialID) {
                        atArr[i] = at;
                        break;
                    }
                }
            }
            return atArr;
        }
    }

    

    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults(limelightName);
    }

    public static enum BreakerLimelightTargetType {
        RETROREFLECTIVE,
        FEDUCIAL,
        BARCODE,
        DETECTOR,
        CLASSIFIER
    }

    public static enum BreakerLimelightLEDMode {
        BLINK,
        ON,
        OFF,
        PIPELINE
    }

    public static enum BreakerLimelightCameraMode {
        VISION_PROCESSOR,
        DRIVER_CAMERA
    }

    public static enum BreakerLimelightStreamMode {
        PICTURE_IN_PICTURE_FOCUS_MAIN,
        PICTURE_IN_PICTURE_FOCUS_SECONDARY,
        SIDE_BY_SIDE
    }

    @Override
    public void runSelfTest() {
        this.health = DeviceHealth.NOMINAL;
    }
  


}
