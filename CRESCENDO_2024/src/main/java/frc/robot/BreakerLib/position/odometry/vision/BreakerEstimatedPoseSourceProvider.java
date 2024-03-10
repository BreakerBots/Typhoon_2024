// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.vision;

import java.util.Optional;
import java.util.function.Function;

// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public interface BreakerEstimatedPoseSourceProvider {

    public abstract BreakerEstimatedPoseSource getEstimatedPoseSource(AprilTagFieldLayout apriltagFieldLayout);
    public abstract BreakerEstimatedPoseSource getEstimatedPoseSource(AprilTagFieldLayout apriltagFieldLayout, Function<BreakerEstimatedPose, Matrix<N3, N1>> stdDevCalculation);
    

    public static interface BreakerEstimatedPoseSource {
        public abstract Optional<BreakerEstimatedPose> getEstimatedPose(PoseOrigin origin);
    }

    public static class PoseCordinateSystem {
        public final Pose3d globalOriginPose;
        public final CoordinateSystem coordinateSystem;
        public static final PoseCordinateSystem GLOBAL = new PoseCordinateSystem(new Pose3d(), CoordinateSystem.NWU());
        /**
         * 
         * @param globalOriginPose The pose in a global refrence frame (wpilib world origin) that woud read as (X:0, Y:0, and theta:0) in the new system
         * @param coordinateSystem 
         */
        public PoseCordinateSystem(Pose3d globalOriginPose, CoordinateSystem coordinateSystem) {
            this.globalOriginPose = globalOriginPose;
            this.coordinateSystem = coordinateSystem;
        }

        public Pose3d toGlobal(Pose3d poseToConvert) {
            Pose3d globalSystemDeltaPose = CoordinateSystem.convert(poseToConvert, coordinateSystem, CoordinateSystem.NWU());
            return globalOriginPose.transformBy(new Transform3d(globalSystemDeltaPose.getTranslation(), globalSystemDeltaPose.getRotation()));
        }

        public Pose3d fromGlobal(Pose3d poseToConvert) {
            Pose3d globalSystemDeltaPose = poseToConvert.relativeTo(globalOriginPose);
            return CoordinateSystem.convert(globalSystemDeltaPose, CoordinateSystem.NWU(), coordinateSystem);
        }

        public static Pose3d convert(Pose3d poseToConvert, PoseCordinateSystem originalSystem, PoseCordinateSystem newSystem) {
           Pose3d globalPose = originalSystem.toGlobal(poseToConvert);
           return newSystem.fromGlobal(globalPose);
        }

        public Pose2d toGlobal(Pose2d poseToConvert) {
            return toGlobal(new Pose3d(poseToConvert.getX(), poseToConvert.getY(), 0.0, new Rotation3d(0.0, 0.0, poseToConvert.getRotation().getRadians()))).toPose2d();
        }

        public Pose2d fromGlobal(Pose2d poseToConvert) {
            return fromGlobal(new Pose3d(poseToConvert.getX(), poseToConvert.getY(), 0.0, new Rotation3d(0.0, 0.0, poseToConvert.getRotation().getRadians()))).toPose2d();
        }

        public static Pose2d convert(Pose2d poseToConvert, PoseCordinateSystem originalSystem, PoseCordinateSystem newSystem) {
           return convert(new Pose3d(poseToConvert.getX(), poseToConvert.getY(), 0.0, new Rotation3d(0.0, 0.0, poseToConvert.getRotation().getRadians())), originalSystem, newSystem).toPose2d();
        }
        
    }

    public static class PoseOrigin {
        public static enum PoseOriginType {
            CORDINATE_SYSTEM,
            ALLIANCE_CORDINATE_SYSTEM,
            GLOBAL
        }
        private PoseOriginType originType;
        private Optional<PoseCordinateSystem> cordinateSystemRed;
        private Optional<PoseCordinateSystem> cordinateSystemBlue;
        private Optional<PoseCordinateSystem> cordinateSystem;

        private PoseOrigin(PoseOriginType originType,
            Optional<PoseCordinateSystem> cordinateSystemRed,
            Optional<PoseCordinateSystem> cordinateSystemBlue,
            Optional<PoseCordinateSystem> cordinateSystem) {
                this.originType = originType;

        }

        private PoseOrigin(PoseOriginType originType) {
            this(originType, Optional.empty(), Optional.empty(), Optional.empty());
        }

        public static PoseOrigin of(PoseCordinateSystem cordinateSystem) {
            return new PoseOrigin(PoseOriginType.CORDINATE_SYSTEM, Optional.empty(), Optional.empty(), Optional.of(cordinateSystem));
        }

        public static PoseOrigin of(PoseCordinateSystem blueCordinateSystem, PoseCordinateSystem redCordinateSystem) {
            return new PoseOrigin(PoseOriginType.ALLIANCE_CORDINATE_SYSTEM, Optional.of(redCordinateSystem), Optional.of(blueCordinateSystem), Optional.empty());
        }

        public static PoseOrigin ofGlobal() {
            return new PoseOrigin(PoseOriginType.GLOBAL);
        }

        public Optional<PoseCordinateSystem> getCordinateSystem() {
            switch (originType) {
                case ALLIANCE_CORDINATE_SYSTEM:
                    Optional<Alliance> ally = DriverStation.getAlliance();
                    if (ally.isPresent()) {
                        if (ally.get() == Alliance.Blue) {
                            return cordinateSystemBlue;
                        } else {
                            return cordinateSystemRed;
                        }
                    } else {
                        return Optional.empty();
                    }
                case CORDINATE_SYSTEM:
                    return cordinateSystem;
                case GLOBAL:
                default:
                    return Optional.of(PoseCordinateSystem.GLOBAL);
                
            }
        }

        public PoseOriginType getOriginType() {
            return originType;
        }

    }
    
    public static class BreakerEstimatedPose {
        public final PoseCordinateSystem poseCordinateSystem;
        public final Pose3d estimatedPose;
        public final double captureTimestamp;
        public final AprilTag[] apriltagsUsed;
        public final Optional<Matrix<N3, N1>> estimationStandardDevations;

        public BreakerEstimatedPose(BreakerEstimatedPose originalEstimatedPose, PoseCordinateSystem newCordinateSystem) {
            this.estimatedPose = PoseCordinateSystem.convert(originalEstimatedPose.estimatedPose, originalEstimatedPose.poseCordinateSystem, newCordinateSystem);
            this.captureTimestamp = originalEstimatedPose.captureTimestamp;
            this. poseCordinateSystem = newCordinateSystem;
            this.apriltagsUsed = originalEstimatedPose.apriltagsUsed;
            this.estimationStandardDevations = originalEstimatedPose.estimationStandardDevations;
        }

        public BreakerEstimatedPose(Pose3d estimatedPose, double captureTimestamp, PoseCordinateSystem  poseCordinateSystem, AprilTag... aprilTagsUsed) {
            this.estimatedPose = estimatedPose;
            this.captureTimestamp = captureTimestamp;
            this.poseCordinateSystem =  poseCordinateSystem;
            this.apriltagsUsed = aprilTagsUsed;
            estimationStandardDevations = Optional.empty();
        }

        public BreakerEstimatedPose(Pose3d estimatedPose, double captureTimestamp, PoseCordinateSystem  poseCordinateSystem, Matrix<N3, N1> estimationStandardDevations, AprilTag... aprilTagsUsed) {
            this.estimatedPose = estimatedPose;
            this.captureTimestamp = captureTimestamp;
            this. poseCordinateSystem = poseCordinateSystem;
            this.apriltagsUsed = aprilTagsUsed;
            this.estimationStandardDevations = Optional.of(estimationStandardDevations);
        }  

        public BreakerEstimatedPose(BreakerEstimatedPose originalEstimate, Matrix<N3, N1> estimationStandardDevations) {
            this.estimatedPose = originalEstimate.estimatedPose;
            this.captureTimestamp = originalEstimate.captureTimestamp;
            this.poseCordinateSystem = originalEstimate.poseCordinateSystem;
            this.apriltagsUsed = originalEstimate.apriltagsUsed;
            this.estimationStandardDevations = Optional.of(estimationStandardDevations);
        }  
    }

    public static class BreakerPoseEstimationStandardDeviationCalculator implements Function<BreakerEstimatedPose, Matrix<N3, N1>> {

        private Matrix<N3, N1> singleTagStdDevs;
        private Matrix<N3, N1> multiTagStdDevs;
        private double maxSingleTagDist, maxMultiTagDist, distanceScaleFactor;

        public BreakerPoseEstimationStandardDeviationCalculator() {
            this(VecBuilder.fill(8, 8, 8), VecBuilder.fill(0.5, 0.5, 1));
        }

        public BreakerPoseEstimationStandardDeviationCalculator(Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs) {
            this(singleTagStdDevs, multiTagStdDevs, 3.5, 6.5, 5.0);
        }

        public BreakerPoseEstimationStandardDeviationCalculator(double maxSingleTagDist, double maxMultiTagDist, double distanceScaleFactor) {
            this(VecBuilder.fill(6, 6, 8), VecBuilder.fill(0.5, 0.5, 1), maxSingleTagDist, maxMultiTagDist, distanceScaleFactor);
        }
        public BreakerPoseEstimationStandardDeviationCalculator(Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs, double maxSingleTagDist, double maxMultiTagDist, double distanceScaleFactor) {
            this.multiTagStdDevs = multiTagStdDevs;
            this.singleTagStdDevs = singleTagStdDevs;
            this.maxSingleTagDist = maxSingleTagDist;
            this.maxMultiTagDist = maxMultiTagDist;
            this.distanceScaleFactor = distanceScaleFactor;
        }
        @Override
        public Matrix<N3, N1> apply(BreakerEstimatedPose t) {
            var estStdDevs = singleTagStdDevs;
            var targets = t.apriltagsUsed;
            int numTags = 0;
            double avgDist = 0;
            for (var tgt : targets) {
                var tagPose = tgt.pose;
                numTags++;
                avgDist +=
                        tagPose.toPose2d().getTranslation().getDistance(t.estimatedPose.getTranslation().toTranslation2d());
            }
            if (numTags == 0) return estStdDevs;
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) estStdDevs = multiTagStdDevs;
            // Increase std devs based on (average) distance
            if ((numTags == 1 && avgDist > maxSingleTagDist) || (numTags > 1 && avgDist > maxMultiTagDist) )//4
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / distanceScaleFactor)); //30

            return estStdDevs;
        }
    }

}
