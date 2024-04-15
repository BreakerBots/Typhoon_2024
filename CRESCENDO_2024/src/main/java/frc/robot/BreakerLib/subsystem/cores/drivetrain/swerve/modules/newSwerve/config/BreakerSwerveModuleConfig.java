// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.newSwerve.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.util.MechanismRatio;

/** Add your docs here. */
public class BreakerSwerveModuleConfig {
    private final BreakerSwerveDriveMotorConfig2 driveMotorConfig;
    private final BreakerSwerveAzimuthMotorConfig2 azimuthMotorConfig;
    private final Rotation2d azimuthEncoderOffset;
    private final MechanismRatio couplingRatio;
    private final Translation2d modulePosition;
    public BreakerSwerveModuleConfig(BreakerSwerveDriveMotorConfig2 driveMotorConfig, BreakerSwerveAzimuthMotorConfig2 azimuthMotorConfig, Rotation2d azimuthEncoderOffset, MechanismRatio couplingRatio, Translation2d modulePosition) {
        this.driveMotorConfig = driveMotorConfig;
        this.azimuthMotorConfig = azimuthMotorConfig;
        this.azimuthEncoderOffset = azimuthEncoderOffset;
        this.couplingRatio = couplingRatio;
        this.modulePosition = modulePosition;
    }
    
    public BreakerSwerveAzimuthMotorConfig2 getAzimuthMotorConfig() {
        return azimuthMotorConfig;
    }

    public Rotation2d getAzimuthEncoderOffset() {
        return azimuthEncoderOffset;
    }

    public MechanismRatio getCouplingRatio() {
        return couplingRatio;
    }

    public BreakerSwerveDriveMotorConfig2 getDriveMotorConfig() {
        return driveMotorConfig;
    }

    public Translation2d getModulePosition() {
        return modulePosition;
    }

    public BreakerSwerveModuleConfig withDriveMotorConfig(BreakerSwerveDriveMotorConfig2 driveMotorConfig) {
        return new BreakerSwerveModuleConfig(driveMotorConfig, azimuthMotorConfig, azimuthEncoderOffset, couplingRatio, modulePosition);
    }

    public BreakerSwerveModuleConfig withAzimuthMotorConfig(BreakerSwerveAzimuthMotorConfig2 azimuthMotorConfig) {
        return new BreakerSwerveModuleConfig(driveMotorConfig, azimuthMotorConfig, azimuthEncoderOffset, couplingRatio, modulePosition);
    }

    public BreakerSwerveModuleConfig withAzimuthEncoderOffset(Rotation2d azimuthEncoderOffset) {
        return new BreakerSwerveModuleConfig(driveMotorConfig, azimuthMotorConfig, azimuthEncoderOffset, couplingRatio, modulePosition);
    }

    public BreakerSwerveModuleConfig withModulePosition(Rotation2d azimuthEncoderOffset) {
        return new BreakerSwerveModuleConfig(driveMotorConfig, azimuthMotorConfig, azimuthEncoderOffset, couplingRatio, modulePosition);
    }

    public BreakerSwerveModuleConfig withDriveMotorInvert(boolean invert) {
        return new BreakerSwerveModuleConfig(driveMotorConfig.withInvert(invert), azimuthMotorConfig, azimuthEncoderOffset, couplingRatio, modulePosition);
    }

    public BreakerSwerveModuleConfig withAzimuthMotorInvert(boolean invert) {
        return new BreakerSwerveModuleConfig(driveMotorConfig, azimuthMotorConfig.withInvert(invert), azimuthEncoderOffset, couplingRatio, modulePosition);
    }
}
