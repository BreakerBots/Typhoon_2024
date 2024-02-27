// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController.AppliedModifierUnits;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;
import frc.robot.subsystems.Drive;

public class TeleopSnapDriveToCardinalHeading extends Command {
  /** Creates a new SnapDriveToCardinal. */
  private SwerveCardinal cardinal;
  private Drive drive;
  private BreakerTeleopSwerveDriveController teleopDriveController;
  private ProfiledPIDController pid;
  private final Timer timer = new Timer();
  public TeleopSnapDriveToCardinalHeading(SwerveCardinal cardinal, Drive drive, BreakerTeleopSwerveDriveController teleopDriveController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cardinal = cardinal;
    this.drive = drive;
    this.teleopDriveController = teleopDriveController;
    pid = DriveConstants.HEADING_SNAP_PID;
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    pid.reset(drive.getOdometryPoseMeters().getRotation().getRadians(), drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);
    teleopDriveController.overrideTurnInput(() -> {return pid.calculate(drive.getOdometryPoseMeters().getRotation().getRadians(), cardinal.getRotation().getRadians());}, AppliedModifierUnits.UNIT_PER_SEC);
    BreakerLog.logEvent(String.format("TeleopSnapDriveToCardinalHeading instance started (goal: %s)", cardinal.toString()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(DriveConstants.HEADING_SNAP_TIMEOUT_SEC)) {
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    teleopDriveController.endAllOverrides();
    if (interrupted) {
      BreakerLog.logEvent(String.format("TeleopSnapDriveToCardinalHeading instance timed out or was cancled, command FAIL, (goal: %s)", cardinal.toString()));
    } else {
      BreakerLog.logEvent(String.format("TeleopSnapDriveToCardinalHeading instance SUCESSFULLY reached its goal and relinquished azimuth control to driver (goal: %s)", cardinal.toString()));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.printf("%.2f, %.2f\n", drive.getOdometryPoseMeters().getRotation().getRadians(), cardinal.getRotation().getRadians());
    return MathUtil.isNear(drive.getOdometryPoseMeters().getRotation().getRadians(), cardinal.getRotation().getRadians(), DriveConstants.HEADING_SNAP_POSITIONAL_TOLERENCE_RAD); //&& Math.abs(drive.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond) <= DriveConstants.HEADING_SNAP_VELOCITY_TOLERENCE_RAD_PER_SEC;
  }

  public static enum SwerveCardinal {
    FRONT(new Rotation2d()),
    LEFT(Rotation2d.fromDegrees(90.0)),
    RIGHT(Rotation2d.fromDegrees(-90.0)),
    BACK(Rotation2d.fromDegrees(180));

    private final Rotation2d rotation;

    SwerveCardinal(Rotation2d rotation) {
        this.rotation = rotation;
    }

    public Rotation2d getRotation() {
        return rotation;
    }
}
}
