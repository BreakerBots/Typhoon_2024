// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
/** Add your docs here. */
public class BreakerSwerveModule extends BreakerGenericSwerveModule {
    private BreakerGenericSwerveModuleDriveMotor driveMotor;
    private BreakerGenericSwerveModuleAngleMotor angleMotor;
    public BreakerSwerveModule(BreakerGenericSwerveModuleDriveMotor driveMotor, BreakerGenericSwerveModuleAngleMotor angleMotor, Translation2d wheelPositionRelativeToRobot) {
        super(wheelPositionRelativeToRobot);
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
    }

    @Override
    public void setModuleTarget(Rotation2d targetAngle, double targetVelocityMetersPerSecond, boolean isOpenLoop) {
        angleMotor.setTargetAngle(targetAngle);
        driveMotor.setTargetVelocity(targetVelocityMetersPerSecond, isOpenLoop);
    }

    @Override
    public Rotation2d getModuleAbsoluteAngle() {
        return angleMotor.getAbsoluteAngle();
    }

    @Override
    public Rotation2d getModuleRelativeAngle() {
        return angleMotor.getRelativeAngle();
    }

    @Override
    public double getModuleVelMetersPerSec() {
        return driveMotor.getVelocity();
    }

    @Override
    public SwerveModuleState getModuleTargetState() {
        return new SwerveModuleState(driveMotor.getVelocity(), angleMotor.getTargetAngle());
    }

    @Override
    public void setDriveMotorBrakeMode(boolean isEnabled) {
        driveMotor.setBrakeMode(isEnabled);
    }

    @Override
    public void setTurnMotorBrakeMode(boolean isEnabled) {
        angleMotor.setBrakeMode(isEnabled);
        
    }

    @Override
    public void setModuleBrakeMode(boolean isEnabled) {
        setDriveMotorBrakeMode(isEnabled);
        setTurnMotorBrakeMode(isEnabled);
    }

    @Override
    public double getModuleDriveDistanceMeters() {
        return driveMotor.getDistance();
    }

    @Override
    public void resetModuleDriveEncoderPosition() {
        driveMotor.resetDistance();
    }

    @Override
    public double getMaxAttainableWheelSpeed() {
        return driveMotor.getConfig().getMaxAttainableWheelSpeed();
    }

    @Override
    public DeviceHealth[] getModuleHealths() {
        DeviceHealth driveHealth = driveMotor.getHealth();
        DeviceHealth turnHealth = angleMotor.getHealth();
        DeviceHealth overall = driveHealth != DeviceHealth.NOMINAL || turnHealth != DeviceHealth.NOMINAL ? DeviceHealth.INOPERABLE : DeviceHealth.NOMINAL;
        return new DeviceHealth[]{overall, driveHealth, turnHealth};
    }

    @Override
    public void runSelfTest() {
        faultStr = "";
        DeviceHealth[] healths = getModuleHealths();
        health = healths[0];
        if (health != DeviceHealth.NOMINAL) {
            if (healths[1] != DeviceHealth.NOMINAL) {
                faultStr += driveMotor.getFaults();
            }
            if (healths[2] != DeviceHealth.NOMINAL) {
                faultStr += angleMotor.getFaults();
            }
        }
    }

    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        angleMotor.toLog(table.getSubtable("AngleMotor"));
        angleMotor.toLog(table.getSubtable("DriveMotor"));
    }

    public static class BreakerSwerveMotorPIDConfig {
        public final double kP, kI, kD, kF;
        public BreakerSwerveMotorPIDConfig(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }
    }

    @Override
    public void setStatusUpdatePeriod(double period) {
        // TODO Auto-generated method stub
        throw ;
    }
}
