// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveAzimuthEncoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.BreakerGenericSwerveModuleAngleMotor.BreakerSwerveModuleAngleMotorConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre.BreakerTalonFXSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.ctre.BreakerProTalonFXSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.angle.rev.BreakerBrushlessSparkMaxSwerveModuleAngleMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.BreakerGenericSwerveModuleDriveMotor.BreakerSwerveModuleDriveMotorConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerProTalonFXSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerTalonFXSwerveModuleDriveMotor.TalonFXControlOutputUnits;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerProTalonFXSwerveModuleDriveMotor.ProTalonFXControlOutputUnits;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerTalonFXSwerveModuleDriveMotor;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.rev.BreakerBrushlessSparkMaxSwerveModuleDriveMotor;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Add your docs here. */
public class BreakerSwerveModuleBuilder {
    private BreakerSwerveModuleConfig config;
    private BreakerGenericSwerveModuleAngleMotor angleMotor;
    private BreakerGenericSwerveModuleDriveMotor driveMotor;
    private BreakerSwerveModuleBuilder(BreakerSwerveModuleConfig config) {
        this.config = config;
    }

    public static BreakerSwerveModuleBuilder getInstance(BreakerSwerveModuleConfig config) {
        return new BreakerSwerveModuleBuilder(config);
    }

    /** Sets the module's angle motor to a CTRE TalonFX with STANDARD LICENCING
     * 
     * @param motor The NON PRO LICENCED TalonFX object to use
     * @param encoder The {@link BreakerSwerveAzimuthEncoder} to use for anguler positioning
     * @param encoderAbsoluteAngleOffsetRotations The offset of the positioning encoder's native reading such that 0rot is forward and the same for all modules 
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withTalonFXAngleMotor(TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetRotations, boolean isMotorInverted) {
        angleMotor = new BreakerTalonFXSwerveModuleAngleMotor(motor, encoder, encoderAbsoluteAngleOffsetRotations, isMotorInverted, config.getAngleMotorConfig());
        return this;
    }

    /** Sets the module's drive motor to a CTRE TalonFX with STANDARD LICENCING
     * 
     * @param motor The NON PRO LICENCED TalonFX object to use
     * @param controlOutputUnits A {@link TalonFXControlOutputUnits} enum that sets the output and calculation units of the motor's velocity closed loop contro. PID and FF units MUST match slected units!
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withTalonFXDriveMotor(TalonFX motor, TalonFXControlOutputUnits controlOutputUnits, boolean isMotorInverted) {
        driveMotor = new BreakerTalonFXSwerveModuleDriveMotor(motor, isMotorInverted, controlOutputUnits, config.getDriveMotorConfig());
        return this;
    }

    
    /** Sets the module's angle motor to a CTRE TalonFX with PRO LICENCING
     * 
     * @param motor The STANDARD LICENCED TalonFX object to use
     * @param encoder The {@link BreakerSwerveAzimuthEncoder} to use for anguler positioning
     * @param encoderAbsoluteAngleOffsetRotations The offset of the positioning encoder's native reading such that 0rot is forward and the same for all modules 
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withProTalonFXAngleMotor(TalonFX motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetRotations, boolean isMotorInverted) {
        angleMotor = new BreakerProTalonFXSwerveModuleAngleMotor(motor, encoder, encoderAbsoluteAngleOffsetRotations, isMotorInverted, config.getAngleMotorConfig());
        return this;
    }

    /** Sets the module's drive motor to a CTRE TalonFX with PRO LICENCING
     * 
     * @param motor The NON PRO LICENCED TalonFX object to use
     * @param controlOutputUnits A {@link ProTalonFXControlOutputUnits} enum that sets the output and calculation units of the motor's velocity closed loop contro. PID and FF units MUST match slected units!
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withProTalonFXDriveMotor(TalonFX motor, ProTalonFXControlOutputUnits controlOutputUnits, boolean isMotorInverted) {
        driveMotor = new BreakerProTalonFXSwerveModuleDriveMotor(motor, isMotorInverted, controlOutputUnits, config.getDriveMotorConfig());
        return this;
    }

    
    /** Sets the module's angle motor to a REV Spark MAX powering a NEO BLDC motor
     * 
     * @param motor The CANSparkMax object to use
     * @param encoder The {@link BreakerSwerveAzimuthEncoder} to use for anguler positioning
     * @param encoderAbsoluteAngleOffsetRotations The offset of the positioning encoder's native reading such that 0rot is forward and the same for all modules 
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withBrushlessSparkMaxAngleMotor(CANSparkMax motor, BreakerSwerveAzimuthEncoder encoder, double encoderAbsoluteAngleOffsetRotations, boolean isMotorInverted) {
        angleMotor = new BreakerBrushlessSparkMaxSwerveModuleAngleMotor(motor, encoder, encoderAbsoluteAngleOffsetRotations, isMotorInverted, config.getAngleMotorConfig());
        return this;
    }

    
    /** Sets the module's drive motor to a REV Spark MAX powering a NEO BLDC motor
     * 
     * @param motor The CANSparkMax object to use
     * @param isMotorInverted Polarity of motor output, used to match motor output to encoder reading
     * @return This {@link BreakerSwerveModuleBuilder} so that config calls can be chained
     */
    public BreakerSwerveModuleBuilder withBrushlessSparkMaxDriveMotor(CANSparkMax motor, boolean isMotorInverted) {
        driveMotor = new BreakerBrushlessSparkMaxSwerveModuleDriveMotor(motor, isMotorInverted, config.getDriveMotorConfig());
        return this;
    }

    /** Creates
     * 
     * @param wheelPositionRelativeToRobot
     * @return
     */
    public BreakerSwerveModule createSwerveModule(Translation2d wheelPositionRelativeToRobot) {
        return new BreakerSwerveModule(driveMotor, angleMotor, wheelPositionRelativeToRobot);
    }

    public static class BreakerSwerveModuleConfig {
        private BreakerSwerveModuleAngleMotorConfig angleMotorConfig;
        private BreakerSwerveModuleDriveMotorConfig driveMotorConfig;
        public BreakerSwerveModuleConfig(double driveGearRatio, double azimuthGearRatio, double wheelDiameter, double maxAttainableWheelSpeed, double angleSupplyCurrentLimit, double driveSupplyCurrentLimit, double angleOutputRampPeriod, double driveOutputRampPeriod, BreakerSwerveMotorPIDConfig anglePIDConfig, BreakerSwerveMotorPIDConfig drivePIDConfig, BreakerArbitraryFeedforwardProvider driveArbFF) {
            angleMotorConfig = new BreakerSwerveModuleAngleMotorConfig(azimuthGearRatio, angleSupplyCurrentLimit, angleOutputRampPeriod, anglePIDConfig);
            driveMotorConfig = new BreakerSwerveModuleDriveMotorConfig(driveGearRatio, wheelDiameter, driveSupplyCurrentLimit, maxAttainableWheelSpeed, driveOutputRampPeriod, driveArbFF, drivePIDConfig);
        }

        public BreakerSwerveModuleConfig(double driveGearRatio, double azimuthGearRatio, double wheelDiameter, double maxAttainableWheelSpeed, double angleSupplyCurrentLimit, double driveSupplyCurrentLimit, BreakerSwerveMotorPIDConfig anglePIDConfig, BreakerSwerveMotorPIDConfig drivePIDConfig, BreakerArbitraryFeedforwardProvider driveArbFF) {
            angleMotorConfig = new BreakerSwerveModuleAngleMotorConfig(azimuthGearRatio, angleSupplyCurrentLimit, anglePIDConfig);
            driveMotorConfig = new BreakerSwerveModuleDriveMotorConfig(driveGearRatio, wheelDiameter, driveSupplyCurrentLimit, maxAttainableWheelSpeed, driveArbFF, drivePIDConfig);
        }

        public BreakerSwerveModuleConfig(BreakerSwerveModuleAngleMotorConfig angleMotorConfig, BreakerSwerveModuleDriveMotorConfig driveMotorConfig) {
            this.angleMotorConfig = angleMotorConfig;
            this.driveMotorConfig = driveMotorConfig;
        }

        public BreakerSwerveModuleConfig withOutputRampPeriods(double angleOutputRampPeriod, double driveOutputRampPeriod) {
            angleMotorConfig.withOutputRampPeriod(angleOutputRampPeriod);
            driveMotorConfig.withOutputRampPeriod(driveOutputRampPeriod);
            return this;
        }

        public BreakerSwerveModuleAngleMotorConfig getAngleMotorConfig() {
            return angleMotorConfig;
        }

        public BreakerSwerveModuleDriveMotorConfig getDriveMotorConfig() {
            return driveMotorConfig;
        }
    }
}
