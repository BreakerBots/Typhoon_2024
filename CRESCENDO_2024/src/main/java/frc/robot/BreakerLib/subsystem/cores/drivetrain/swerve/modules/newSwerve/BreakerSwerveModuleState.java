// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.newSwerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/** Add your docs here. */
public class BreakerSwerveModuleState {
    public final Measure<Distance> driveDistance;
    public final Measure<Velocity<Distance>> driveVelocity;
    public final Rotation2d azimuthAngle;
    public final Measure<Velocity<Angle>> azimuthVelocity;
    private final SwerveModuleState moduleState;
    private final SwerveModulePosition modulePosition;

    public BreakerSwerveModuleState(Measure<Distance> driveDistance, Measure<Velocity<Distance>> driveVelocity, Rotation2d azimuthAngle, Measure<Velocity<Angle>> azimuthVelocity) {
        this.driveDistance = driveDistance;
        this.driveVelocity = driveVelocity;
        this.azimuthAngle = azimuthAngle;
        this.azimuthVelocity = azimuthVelocity;
        this.moduleState = new SwerveModuleState(driveVelocity, azimuthAngle);
        this.modulePosition = new SwerveModulePosition(driveDistance, azimuthAngle);
    }

    public SwerveModuleState toSwerveModuleState() {
        return moduleState;
    }

    public SwerveModulePosition toSwerveModulePosition() {
        return modulePosition;
    }
}
