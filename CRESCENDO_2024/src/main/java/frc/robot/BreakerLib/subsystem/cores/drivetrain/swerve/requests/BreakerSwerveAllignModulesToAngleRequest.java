// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveRequest;

public class BreakerSwerveAllignModulesToAngleRequest implements BreakerSwerveRequest {
    protected Rotation2d angle;
    public BreakerSwerveAllignModulesToAngleRequest(Rotation2d angle) {
      this.angle = angle;
    }

    @Override
    public void apply(BreakerSwerveDrive drivetrain) {
      SwerveModuleState[] requestStates = new SwerveModuleState[drivetrain.getSwerveModulePositions().length];
      for (int i = 0; i < drivetrain.getSwerveModulePositions().length; i++) {
        requestStates[i] = new SwerveModuleState(0.0, angle);
      }
      applyModuleStates(drivetrain, true, false, requestStates);
    }
  }
