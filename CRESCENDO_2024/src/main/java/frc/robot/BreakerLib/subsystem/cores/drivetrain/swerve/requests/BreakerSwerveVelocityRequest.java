package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;

public class BreakerSwerveVelocityRequest extends BreakerGenericSwerveMovementRequest<BreakerSwerveVelocityRequest> {
    protected ChassisSpeeds speeds;
    public BreakerSwerveVelocityRequest(ChassisSpeeds speeds) {
      this(speeds, SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITH_OFFSET, SlowModeValue.DEFAULT, new Translation2d(), 0.02, true, false);
    }

    public BreakerSwerveVelocityRequest(ChassisSpeeds speeds, SwerveMovementRefrenceFrame movementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, double descreteTimestep, boolean headingCorrectionEnabled, boolean isOpenLoop) {
      super(movementRefrenceFrame, slowModeValue, centerOfRotation, descreteTimestep, isOpenLoop, headingCorrectionEnabled);
      this.speeds = speeds;
    }

    @Override
    public BreakerSwerveVelocityRequest withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame) {
      this.swerveMovementRefrenceFrame = swerveMovementRefrenceFrame;
      return this;
    }

    @Override
    public BreakerSwerveVelocityRequest withSlowModeValue(SlowModeValue slowModeValue) {
      this.slowModeValue = slowModeValue;
      return this;
    }

    public BreakerSwerveVelocityRequest withChassisSpeeds(ChassisSpeeds speeds) {
      this.speeds = speeds;
      return this;
    }

    @Override
    public BreakerSwerveVelocityRequest withCenerOfRotation(Translation2d centerOfRotation) {
      this.centerOfRotation = centerOfRotation;
      return this;
    }

    public BreakerSwerveVelocityRequest withVelocityX(double velocityX) {
      speeds.vxMetersPerSecond = velocityX;
      return this;
    }

    public BreakerSwerveVelocityRequest withVelocityY(double velocityY) {
      speeds.vyMetersPerSecond = velocityY;
      return this;
    }

    public BreakerSwerveVelocityRequest withVelocityOmega(double velocityOmega) {
      speeds.omegaRadiansPerSecond = velocityOmega;
      return this;
    }

    @Override
    public BreakerSwerveVelocityRequest withIsOpenLoop(boolean isOpenLoop) {
      this.isOpenLoop = isOpenLoop;
      return this;
    }

    public double getVelocityX() {
      return speeds.vxMetersPerSecond;
    }

    public double getVelocityY() {
      return speeds.vyMetersPerSecond;
    }

    public double getVelocityOmega() {
      return speeds.omegaRadiansPerSecond;
    }

    @Override
    public ChassisSpeeds getRequestedChassisSpeeds(BreakerSwerveDrive drivetrain) {
      return speeds;
    }

    @Override
    public BreakerSwerveVelocityRequest withHeadingCorrectionEnabled(boolean isEnabled) {
      this.headingCorrectionEnabled = isEnabled;
      return this;
    }

    @Override
    public BreakerSwerveVelocityRequest withDescreteTimestep(double descreteTimestep) {
      this.descreteTimestep = descreteTimestep;
      return this;
    }
  }