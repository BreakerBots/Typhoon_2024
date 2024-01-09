// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests;

import javax.sound.midi.SysexMessage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;

/** Add your docs here. */

public class BreakerSwervePercentSpeedRequest extends BreakerGenericSwerveMovementRequest<BreakerSwervePercentSpeedRequest> {
    protected ChassisPercentSpeeds percentSpeeds;

    public BreakerSwervePercentSpeedRequest(ChassisPercentSpeeds percentSpeeds) {
      this(percentSpeeds, SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT, new Translation2d(), 0.02, true, false);
    }

    public BreakerSwervePercentSpeedRequest(ChassisPercentSpeeds percentSpeeds, SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, double descreteTimestep, boolean headingCorrectionEnabled, boolean isOpenLoop) {
      super(movementRefrenceFrame, slowModeValue, centerOfRotation, descreteTimestep, headingCorrectionEnabled, isOpenLoop);
      this.percentSpeeds = percentSpeeds;
    }

    @Override
    public BreakerSwervePercentSpeedRequest withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame) {
      this.swerveMovementRefrenceFrame = swerveMovementRefrenceFrame;
      return this;
    }

    @Override
    public BreakerSwervePercentSpeedRequest withSlowModeValue(SlowModeValue slowModeValue) {
      this.slowModeValue = slowModeValue;
      return this;
    }

    @Override
    public BreakerSwervePercentSpeedRequest withCenerOfRotation(Translation2d centerOfRotation) {
      this.centerOfRotation = centerOfRotation;
      return this;
    }

    public BreakerSwervePercentSpeedRequest withPercentX(double percentX) {
      percentSpeeds.vxPercentOfMax = MathUtil.clamp(percentX, -1.0, 1.0);
      return this;
    }

    public BreakerSwervePercentSpeedRequest withPercentY(double percentY) {
      percentSpeeds.vyPercentOfMax = MathUtil.clamp(percentY, -1.0, 1.0);
      return this;
    }

    public BreakerSwervePercentSpeedRequest withPercentOmega(double percentOmega) {
      percentSpeeds.omegaPercentOfMax = MathUtil.clamp(percentOmega, -1.0, 1.0);
      return this;
    }

    public BreakerSwervePercentSpeedRequest withChassisPercentSpeeds(ChassisPercentSpeeds percentSpeeds) {
      this.percentSpeeds = percentSpeeds;
      return this;
    }
    
    @Override
    public BreakerSwervePercentSpeedRequest withIsOpenLoop(boolean isOpenLoop) {
      this.isOpenLoop = isOpenLoop;
      return this;
    }

    @Override
    public BreakerSwervePercentSpeedRequest withHeadingCorrectionEnabled(boolean isEnabled) {
      this.headingCorrectionEnabled = isEnabled;
      return this;
    }

    @Override
    public BreakerSwervePercentSpeedRequest withDescreteTimestep(double descreteTimestep) {
      this.descreteTimestep = descreteTimestep;
      return this;
    }

    public double getPercentX() {
      return percentSpeeds.vxPercentOfMax;
    }

    public double getPercentY() {
      return percentSpeeds.vyPercentOfMax;
    }

    public double getPercentOmega() {
      return percentSpeeds.omegaPercentOfMax;
    }

    @Override
    public ChassisSpeeds getRequestedChassisSpeeds(BreakerSwerveDrive drivetrain) {
      return percentSpeeds.toChassisSpeeds(drivetrain.getConfig().getMaxLinearVel(), drivetrain.getConfig().getMaxAngleVel());
    }

    public static class ChassisPercentSpeeds {
      /** Represents forward velocity w.r.t the robot frame of reference. (Fwd is +) */
      public double vxPercentOfMax;
    
      /** Represents sideways velocity w.r.t the robot frame of reference. (Left is +) */
      public double vyPercentOfMax;
    
      /** Represents the angular velocity of the robot frame. (CCW is +) */
      public double omegaPercentOfMax;
    
      /** Constructs a ChassisSpeeds with zeros for dx, dy, and theta. */
      public ChassisPercentSpeeds() {}
    
      /**
       * Constructs a ChassisSpeeds object.
       *
       * @param vxPercentOfMax Forward velocity.
       * @param vyPercentOfMax Sideways velocity.
       * @param omegaPercentOfMax Angular velocity.
       */
      public ChassisPercentSpeeds(
          double vxPercentOfMax, double vyPercentOfMax, double omegaPercentOfMax) {
        this.vxPercentOfMax = MathUtil.clamp(vxPercentOfMax, -1.0, 1.0);
        this.vyPercentOfMax = MathUtil.clamp(vyPercentOfMax, -1.0, 1.0);
        this.omegaPercentOfMax = MathUtil.clamp(omegaPercentOfMax, -1.0, 1.0);
      }

      public ChassisPercentSpeeds(ChassisSpeeds chassisSpeeds, double maxLinearVel, double maxAngVel) {
        BreakerVector2 linearVelVec = new BreakerVector2(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).clampMagnitude(0.0, maxLinearVel).div(maxLinearVel);
        vxPercentOfMax = linearVelVec.getX();
        vyPercentOfMax = linearVelVec.getY();
        omegaPercentOfMax =  MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -maxAngVel, maxAngVel) / maxAngVel;
      }

    
      /**
       * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
       * object.
       *
       * @param vxPercentOfMax The component of speed in the x direction relative to the field.
       *     Positive x is away from your alliance wall.
       * @param vyPercentOfMax The component of speed in the y direction relative to the field.
       *     Positive y is to your left when standing behind the alliance wall.
       * @param omegaPercentOfMax The angular rate of the robot.
       * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
       *     considered to be zero when it is facing directly away from your alliance station wall.
       *     Remember that this should be CCW positive.
       * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
       */
      public static ChassisPercentSpeeds fromFieldRelativeSpeeds(
          double vxPercentOfMax,
          double vyPercentOfMax,
          double omegaPercentOfMax,
          Rotation2d robotAngle) {
        return new ChassisPercentSpeeds(
            MathUtil.clamp(vxPercentOfMax, -1.0, 1.0) * robotAngle.getCos() + MathUtil.clamp(vyPercentOfMax, -1.0, 1.0) * robotAngle.getSin(),
            -MathUtil.clamp(vxPercentOfMax, -1.0, 1.0) * robotAngle.getSin() + MathUtil.clamp(vyPercentOfMax, -1.0, 1.0) * robotAngle.getCos(),
            omegaPercentOfMax);
      }
    
      /**
       * Converts a user provided field-relative ChassisSpeeds object into a robot-relative
       * ChassisSpeeds object.
       *
       * @param fieldRelativeSpeeds The ChassisSpeeds object representing the speeds in the field frame
       *     of reference. Positive x is away from your alliance wall. Positive y is to your left when
       *     standing behind the alliance wall.
       * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
       *     considered to be zero when it is facing directly away from your alliance station wall.
       *     Remember that this should be CCW positive.
       * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
       */
      public static ChassisPercentSpeeds fromFieldRelativeSpeeds(
          ChassisPercentSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
        return fromFieldRelativeSpeeds(
            fieldRelativeSpeeds.vxPercentOfMax,
            fieldRelativeSpeeds.vyPercentOfMax,
            fieldRelativeSpeeds.omegaPercentOfMax,
            robotAngle);
      }
    
      @Override
      public String toString() {
        return String.format(
            "ChassisSpeeds(Vx: %.2f (% of max), Vy: %.2f (% of max), Omega: %.2f (% of max))",
            vxPercentOfMax, vyPercentOfMax, omegaPercentOfMax);
      }

      public ChassisSpeeds toChassisSpeeds(double maxLinearVel, double maxAngVel) {
        double clampedX = MathUtil.clamp(vxPercentOfMax, -1.0, 1.0);
        double clampedY = MathUtil.clamp(vyPercentOfMax, -1.0, 1.0);
        double clampedOmega = MathUtil.clamp(omegaPercentOfMax, -1.0, 1.0);
        BreakerVector2 linearVelVec = new BreakerVector2(clampedX, clampedY).clampMagnitude(0.0, 1.0).times(maxLinearVel);
        return new ChassisSpeeds(linearVelVec.getX(), linearVelVec.getY(), clampedOmega * maxAngVel);
      }

      
    }

  }
