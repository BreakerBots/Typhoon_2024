// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerProfiledHolonomicDriveControler {
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController angleController;
    private Pose2d curPose = new Pose2d(), tgtPose = new Pose2d(), tolerences = new Pose2d();
    private boolean calculateHasBeenRun = false;

    public BreakerProfiledHolonomicDriveControler(ProfiledPIDController xController, ProfiledPIDController yController, ProfiledPIDController angleController) {
        this.angleController = angleController;
        this.xController = xController;
        this.yController = yController;
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setTolerances(Pose2d tolerences) {
        this.tolerences = tolerences;
    }

    private void setConstraints(Pose2d goal, Pose2d currentPose, Constraints linearConstraints, Constraints angularConstraints) {;
        Rotation2d robotToGoalAng = tgtPose.getTranslation().minus(curPose.getTranslation()).getAngle();
        BreakerVector2 maxVelVec = new BreakerVector2(robotToGoalAng, linearConstraints.maxVelocity).abs();
        BreakerVector2 maxAccelVec = new BreakerVector2(robotToGoalAng, linearConstraints.maxAcceleration).abs();
        xController.setConstraints(new Constraints(maxVelVec.getX(), maxAccelVec.getX()));
        yController.setConstraints(new Constraints(maxVelVec.getY(), maxAccelVec.getY()));
        angleController.setConstraints(angularConstraints);

    }

    /**
     * 
     * @param currentPose Robot's current pose.
     * @param targetPose Robot's target pose.
     * @param maxLinearVelocity Maximum linear velocity of pose.
     * @return Robot chassis speeds for swerve drive.
     */
    public ChassisSpeeds calculate(Pose2d goal, Pose2d currentPose, Constraints linearConstraints, Constraints angularConstraints) {
        tgtPose = goal;
        curPose = currentPose;
        calculateHasBeenRun = true;
        setConstraints(goal, currentPose, linearConstraints, angularConstraints);
        double xVel = xController.calculate(currentPose.getX(), goal.getX());
        double yVel = yController.calculate(currentPose.getY(), goal.getY());
        BreakerVector2 linVec = new BreakerVector2(xVel, yVel);
        linVec = new BreakerVector2(linVec.getVectorRotation(), MathUtil.clamp(linVec.getMagnitude(), -linearConstraints.maxVelocity, linearConstraints.maxVelocity));
        double aVel = angleController.calculate(currentPose.getRotation().getRadians(), goal.getRotation().getRadians());
        return new ChassisSpeeds(linVec.getX(), linVec.getY(), aVel);
    }

    public boolean atTargetPose() {
        return  BreakerMath.epsilonEquals(tgtPose.getX(), curPose.getX(), tolerences.getX()) &&
                BreakerMath.epsilonEquals(tgtPose.getY(), curPose.getY(), tolerences.getY()) &&
                BreakerMath.epsilonEquals(tgtPose.getRotation().getRadians(), curPose.getRotation().getRadians(), tolerences.getRotation().getRadians()) &&
                calculateHasBeenRun;
    }

    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        angleController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
        xController.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
        yController.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    }


}
