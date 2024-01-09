// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveRequest;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;

public class BreakerSwerveStopRequest implements BreakerSwerveRequest {

    public BreakerSwerveStopRequest() {}

    @Override
    public void apply(BreakerSwerveDrive drivetrain) {
      applyChassisSpeeds(drivetrain, new ChassisSpeeds(), SwerveMovementRefrenceFrame.ROBOT_RELATIVE, SlowModeValue.DEFAULT,  new Translation2d(), 0.0, false, false);
    }
  }