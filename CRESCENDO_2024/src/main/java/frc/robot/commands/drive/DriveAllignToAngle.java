// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerTeleopSwerveDriveController.AppliedModifierUnits;
import frc.robot.subsystems.Drive;

public class DriveAllignToAngle extends Command {
  /** Creates a new DriveSnapToAngle. */
  private Drive drivetrain;
  private Supplier<Rotation2d> targetAngleSupplier;
  private double allowablePositionError;
  private double allowableVelocityError;
  private ProfiledPIDController anglePID;
  private BreakerTeleopSwerveDriveController driveController;
  public DriveAllignToAngle(Drive drivetrain, BreakerTeleopSwerveDriveController driveController, double allowablePositionError, double allowableVelocityError, TrapezoidProfile.Constraints snapConstraints, Supplier<Rotation2d> targetAngleSupplier) {
    anglePID = new ProfiledPIDController(0.0, 0.0, 0.0, snapConstraints);
    anglePID.setTolerance(allowablePositionError, allowableVelocityError);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglePID.reset(drivetrain.getOdometryPoseMeters().getRotation().getRadians(), drivetrain.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond);
    driveController.overrideTurnInput(this::calculateFeedback, AppliedModifierUnits.UNIT_PER_SEC);
  }

  private double calculateFeedback() {
    return anglePID.calculate(drivetrain.getOdometryPoseMeters().getRotation().getRadians(), targetAngleSupplier.get().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
