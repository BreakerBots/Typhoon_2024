// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerSwerveRequest;

public class BreakerSwerveModuleStateRequest implements BreakerSwerveRequest {
    protected boolean applyRawStates;
    protected boolean isOpenLoop;
    protected SwerveModuleState[] moduleStates;

    public BreakerSwerveModuleStateRequest(boolean applyRawStates, boolean isOpenLoop, SwerveModuleState... moduleStates) {
      this.applyRawStates = applyRawStates;
      this.moduleStates = moduleStates;
      this.isOpenLoop = isOpenLoop;
    }

    public boolean getApplyRawStates() {
        return applyRawStates;
    }

    public SwerveModuleState[] getRequestedModuleStates() {
        return moduleStates;
    }

    public BreakerSwerveModuleStateRequest withApplyRawStates(boolean applyRawStates) {
      this.applyRawStates = applyRawStates;
      return this;
    }

    public BreakerSwerveModuleStateRequest withModuleStates(SwerveModuleState... moduleStates) {
      this.moduleStates = moduleStates;
      return this;
    }

    public BreakerSwerveModuleStateRequest withIsOpenLoop(boolean isOpenLoop) {
      this.isOpenLoop = isOpenLoop;
      return this;
    }

    @Override
    public void apply(BreakerSwerveDrive drivetrain) {
      applyModuleStates(drivetrain, applyRawStates, isOpenLoop, moduleStates);
    }
  }

