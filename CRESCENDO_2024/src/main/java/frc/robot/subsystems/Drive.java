// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.AUTO_CONFIG;
import static frc.robot.Constants.DriveConstants.BL_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.BL_ENCODER_ID;
import static frc.robot.Constants.DriveConstants.BL_ENCODER_OFFSET;
import static frc.robot.Constants.DriveConstants.BL_TRANSLATION;
import static frc.robot.Constants.DriveConstants.BL_TURN_ID;
import static frc.robot.Constants.DriveConstants.BR_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.BR_ENCODER_ID;
import static frc.robot.Constants.DriveConstants.BR_ENCODER_OFFSET;
import static frc.robot.Constants.DriveConstants.BR_TRANSLATION;
import static frc.robot.Constants.DriveConstants.BR_TURN_ID;
import static frc.robot.Constants.DriveConstants.DRIVE_BASE_CONFIG;
import static frc.robot.Constants.DriveConstants.FL_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.FL_ENCODER_ID;
import static frc.robot.Constants.DriveConstants.FL_ENCODER_OFFSET;
import static frc.robot.Constants.DriveConstants.FL_TRANSLATION;
import static frc.robot.Constants.DriveConstants.FL_TURN_ID;
import static frc.robot.Constants.DriveConstants.FR_DRIVE_ID;
import static frc.robot.Constants.DriveConstants.FR_ENCODER_ID;
import static frc.robot.Constants.DriveConstants.FR_ENCODER_OFFSET;
import static frc.robot.Constants.DriveConstants.FR_TRANSLATION;
import static frc.robot.Constants.DriveConstants.FR_TURN_ID;
import static frc.robot.Constants.DriveConstants.MODULE_CONFIG;
import static frc.robot.Constants.DriveConstants.ODOMETRY_CONFIG;
import static frc.robot.Constants.GeneralConstants.DRIVE_CANIVORE_NAME;

import org.littletonrobotics.junction.LogTable;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.BreakerLib.devices.sensors.imu.ctre.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerPhoenixTimesyncSwerveOdometryThread;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveOdometryThread;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerPhoenixTimesyncSwerveOdometryThread.CTREGyroYawStatusSignals;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerPhoenixTimesyncSwerveOdometryThread.CTRESwerveModuleStatusSignals;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.encoders.BreakerSwerveCANcoder;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.motors.drive.ctre.BreakerProTalonFXSwerveModuleDriveMotor.ProTalonFXControlOutputUnits;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

/** Add your docs here. */
public class Drive extends BreakerSwerveDrive {


    private static TalonFX driveFL = new TalonFX(FL_DRIVE_ID, DRIVE_CANIVORE_NAME);
    private static TalonFX turnFL = new TalonFX(FL_TURN_ID, DRIVE_CANIVORE_NAME);
    private static BreakerSwerveCANcoder encoderFL = new BreakerSwerveCANcoder(new CANcoder(FL_ENCODER_ID, DRIVE_CANIVORE_NAME));

    private static TalonFX driveFR = new TalonFX(FR_DRIVE_ID, DRIVE_CANIVORE_NAME);
    private static TalonFX turnFR = new TalonFX(FR_TURN_ID, DRIVE_CANIVORE_NAME);
    private static BreakerSwerveCANcoder encoderFR =  new BreakerSwerveCANcoder(new CANcoder(FR_ENCODER_ID, DRIVE_CANIVORE_NAME));

    private static TalonFX driveBL = new TalonFX(BL_DRIVE_ID, DRIVE_CANIVORE_NAME);
    private static TalonFX turnBL = new TalonFX(BL_TURN_ID, DRIVE_CANIVORE_NAME);
    private static BreakerSwerveCANcoder encoderBL =  new BreakerSwerveCANcoder(new CANcoder(BL_ENCODER_ID, DRIVE_CANIVORE_NAME));

    private static TalonFX driveBR = new TalonFX(BR_DRIVE_ID, DRIVE_CANIVORE_NAME);
    private static TalonFX turnBR = new TalonFX(BR_TURN_ID, DRIVE_CANIVORE_NAME);
    private static BreakerSwerveCANcoder encoderBR =  new BreakerSwerveCANcoder(new CANcoder(BR_ENCODER_ID, DRIVE_CANIVORE_NAME));

    private static BreakerSwerveModule frontLeftModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withProTalonFXAngleMotor(turnFL, encoderFL, FL_ENCODER_OFFSET, true)
        .withProTalonFXDriveMotor(driveFL, ProTalonFXControlOutputUnits.VOLTAGE, true)
        .createSwerveModule(FL_TRANSLATION);

    private static BreakerSwerveModule frontRightModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withProTalonFXAngleMotor(turnFR, encoderFR, FR_ENCODER_OFFSET, true)
        .withProTalonFXDriveMotor(driveFR, ProTalonFXControlOutputUnits.VOLTAGE, false)
        .createSwerveModule(FR_TRANSLATION);

    private static BreakerSwerveModule backLeftModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withProTalonFXAngleMotor(turnBL, encoderBL, BL_ENCODER_OFFSET, true)
        .withProTalonFXDriveMotor(driveBL, ProTalonFXControlOutputUnits.VOLTAGE, true)
        .createSwerveModule(BL_TRANSLATION);

    private static BreakerSwerveModule backRightModule = BreakerSwerveModuleBuilder.getInstance(MODULE_CONFIG)
        .withProTalonFXAngleMotor(turnBR, encoderBR, BR_ENCODER_OFFSET, true)
        .withProTalonFXDriveMotor(driveBR, ProTalonFXControlOutputUnits.VOLTAGE, false)
        .createSwerveModule(BR_TRANSLATION);

    private static Field2d field = new Field2d();
    

    public Drive(BreakerPigeon2 pigeon) {
        super(
            DRIVE_BASE_CONFIG, 
            AUTO_CONFIG, 
            // new BreakerPhoenixTimesyncSwerveOdometryThread(
            //     ODOMETRY_CONFIG, 
            //     new CTREGyroYawStatusSignals((pigeon.getBaseGyro()).getYaw(), pigeon.getBaseGyro().getAngularVelocityZWorld()),
            //     new CTRESwerveModuleStatusSignals(driveFL.getRotorPosition(), driveFL.getRotorVelocity(), encoderFL.getBaseEncoder().getAbsolutePosition(), encoderFL.getBaseEncoder().getVelocity(), MODULE_CONFIG.getDriveMotorConfig().getWheelDiameter() * Math.PI),
            //     new CTRESwerveModuleStatusSignals(driveFR.getRotorPosition(), driveFR.getRotorVelocity(), encoderFR.getBaseEncoder().getAbsolutePosition(), encoderFR.getBaseEncoder().getVelocity(), MODULE_CONFIG.getDriveMotorConfig().getWheelDiameter() * Math.PI),
            //     new CTRESwerveModuleStatusSignals(driveBL.getRotorPosition(), driveBL.getRotorVelocity(), encoderBL.getBaseEncoder().getAbsolutePosition(), encoderBL.getBaseEncoder().getVelocity(), MODULE_CONFIG.getDriveMotorConfig().getWheelDiameter() * Math.PI),
            //     new CTRESwerveModuleStatusSignals(driveBR.getRotorPosition(), driveBR.getRotorVelocity(), encoderBR.getBaseEncoder().getAbsolutePosition(), encoderBR.getBaseEncoder().getVelocity(), MODULE_CONFIG.getDriveMotorConfig().getWheelDiameter() * Math.PI)
            // ),
            new BreakerSwerveOdometryThread(ODOMETRY_CONFIG),
            pigeon, 
            frontLeftModule, 
            frontRightModule,
            backLeftModule, 
            backRightModule
        );
        // BreakerDashboard.getMainTab().add(field);
        
        frontLeftModule.setDeviceName("FL_Module");
        frontRightModule.setDeviceName("FR_Module");
        backLeftModule.setDeviceName("BL_Module");
        backRightModule.setDeviceName("BR_Module");

        // BreakerDashboard.getDiagnosticsTab().add("FL Module", frontLeftModule);
        // BreakerDashboard.getDiagnosticsTab().add("FR Module", frontRightModule);
        // BreakerDashboard.getDiagnosticsTab().add("BL Module", backLeftModule);
        // BreakerDashboard.getDiagnosticsTab().add("BR Module", backRightModule);

        BreakerLog.registerLogable("Drive", this);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
    }
}
