// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import org.apache.commons.math3.analysis.function.Constant;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterTarget.FireingSolution;
import frc.robot.Constants;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveVelocityRequest;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class AimToTargetStationary extends Command {
  /** Creates a new AimDriveToSpeaker. */

  private ProfiledPIDController anglePID;
  private Shooter shooter;
  private Drive drive;
  private BreakerSwerveVelocityRequest velocityRequest;
  private final Timer timer = new Timer();
  public AimToTargetStationary(Shooter shooter, Drive drive) {
    anglePID = new ProfiledPIDController(2.25, 0.0, 0.1, new TrapezoidProfile.Constraints(/*6.0*/Constants.DriveConstants.MAX_ANGULAR_VEL, /*6.0*/Constants.DriveConstants.MAX_ANGULAR_VEL * Constants.DriveConstants.MAX_ANGULAR_VEL));
    anglePID.setTolerance(Math.toRadians(0.3), Math.toRadians(10.0));
    anglePID.enableContinuousInput(-Math.PI, Math.PI);
    this.drive = drive;
    this.shooter = shooter;
    velocityRequest = new BreakerSwerveVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED, new Translation2d(), 0.02, false, false);
    addRequirements(drive, shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    anglePID.reset(drive.getOdometryPoseMeters().getRotation().getRadians());
    shooter.setState(ShooterState.TRACK_TARGET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    FireingSolution fireingSolution = shooter.getActiveTargetFireingSolution();
    BreakerLog.recordOutput("ATT: FLY GOL", shooter.isAtFlywheelGoal());
    BreakerLog.recordOutput("ATT: SHT ANG GOL", shooter.isAtAngleGoal());
    BreakerLog.recordOutput("ATT: RBT ANG GOL", anglePID.atSetpoint());
    System.out.println(fireingSolution.fireingVec());
    System.out.println(!anglePID.atSetpoint());
    double omegaDemand = anglePID.calculate(drive.getOdometryPoseMeters().getRotation().getRadians(), fireingSolution.yaw().getRadians());
    if (!anglePID.atSetpoint()) {
      drive.applyRequest(velocityRequest.withChassisSpeeds(new ChassisSpeeds(0.0, 0.0, omegaDemand)));
    } else {
      drive.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.isAtGoal() && anglePID.atSetpoint()) || timer.hasElapsed(3.0);
  }
}
