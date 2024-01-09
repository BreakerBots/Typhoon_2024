package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;

public abstract class BreakerGenericSwerveMovementRequest<T extends BreakerGenericSwerveMovementRequest<T>> implements BreakerSwerveRequest {
    protected SwerveMovementRefrenceFrame swerveMovementRefrenceFrame;
    protected SlowModeValue slowModeValue;
    protected boolean headingCorrectionEnabled;
    protected Translation2d centerOfRotation;
    protected boolean isOpenLoop;
    protected double descreteTimestep;
    protected BreakerGenericSwerveMovementRequest(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame, SlowModeValue slowModeValue, Translation2d centerOfRotation, double descreteTimestep, boolean headingCorrectionEnabled, boolean isOpenLoop) {
      this.slowModeValue = slowModeValue;
      this.swerveMovementRefrenceFrame = swerveMovementRefrenceFrame;
      this.centerOfRotation = centerOfRotation;
      this.headingCorrectionEnabled = headingCorrectionEnabled;
      this.isOpenLoop = isOpenLoop;
      this.descreteTimestep = descreteTimestep;
    } 

    public abstract T withSwerveMovementRefrenceFrame(SwerveMovementRefrenceFrame swerveMovementRefrenceFrame);

    /** Sets whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
     * @param slowModeValue Whether or not you want to apply the drivetrians slow mode scailar or simply default to the global setting.
     * @return this.
     */
    public abstract T withSlowModeValue(SlowModeValue slowModeValue);

    public abstract T withCenerOfRotation(Translation2d centerOfRotation);

    public abstract T withIsOpenLoop(boolean isOpenLoop);

    public abstract T withHeadingCorrectionEnabled(boolean isEnabled);

    public abstract T withDescreteTimestep(double descreteTimestep);

    public SlowModeValue getSlowModeValue() {
        return slowModeValue;
    }

    public SwerveMovementRefrenceFrame getSwerveMovementRefrenceFrame() {
        return swerveMovementRefrenceFrame;
    }

    public Translation2d getCenterOfRotation() {
        return centerOfRotation;
    }

    public boolean getHeadingCorrectionEnabled() {
        return headingCorrectionEnabled;
    }

    public double getDescreteTimestep() {
        return descreteTimestep;
    }

    public abstract ChassisSpeeds getRequestedChassisSpeeds(BreakerSwerveDrive drivetrain);

    @Override
    public void apply(BreakerSwerveDrive drivetrain) {
     applyChassisSpeeds(drivetrain, getRequestedChassisSpeeds(drivetrain), swerveMovementRefrenceFrame, slowModeValue, centerOfRotation, descreteTimestep, headingCorrectionEnabled, isOpenLoop); 
    }
  }