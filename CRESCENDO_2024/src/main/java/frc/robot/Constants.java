
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.ShooterTarget.FireingSolution;
import frc.robot.ShooterTarget.FireingTableValue;
import frc.robot.ShooterTarget.SmartSpoolConfig;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.position.odometry.swerve.BreakerSwerveOdometryThread.BreakerSwerveOdometryConfig;
import frc.robot.BreakerLib.position.odometry.vision.BreakerEstimatedPoseSourceProvider.PoseOrigin;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain.SlowModeValue;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.BreakerPathplannerSwerveAutoConfig.BreakerPathplannerStandardSwerveAutoConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive.SwerveMovementRefrenceFrame;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModule.BreakerSwerveMotorPIDConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules.BreakerSwerveModuleBuilder.BreakerSwerveModuleConfig;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.requests.BreakerSwerveVelocityRequest;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolablePair;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerInterpolatingTreeMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class GeneralConstants {
    public static final String DRIVE_CANIVORE_NAME = "drive_canivore";
  }
  public static class FieldConstants {
    public static final double FIELD_WIDTH = 16.541;//16.4846
    public static final Translation3d BLUE_SPEAKER_AIM_POINT = new Translation3d(-0.038099999999999995, 5.547867999999999, 1.4511020000000001).plus(new Translation3d(Units.inchesToMeters(-10.0), Units.inchesToMeters(0), Units.inchesToMeters(30.322)));
    public static final Translation3d RED_SPEAKER_AIM_POINT = new Translation3d( 16.579342, 5.547867999999999, 1.4511020000000001).plus(new Translation3d(Units.inchesToMeters(10.0), Units.inchesToMeters(0), Units.inchesToMeters(30.322)));
  }

  public static class IntakeConstants {
    public static final int ROLLER_MOTOR_ID = 40; 
    public static final int PIVOT_LEFT_ID = 41;
    public static final int PIVOT_RIGHT_ID = 42;
    public static final int PIVOT_ENCODER_ID = 43;
    
    public static final double PIVOT_ENCODER_OFFSET = 0.2822265625;


    public static final Rotation2d PIVIOT_RETRACTED_THRESHOLD = Rotation2d.fromRotations(0.32);//0.32;
    public static final Rotation2d PIVOT_RETRACTED_TARGET = Rotation2d.fromRotations(0.325);
    
    public static final Rotation2d PIVIOT_EXTENDED_THRESHOLD = Rotation2d.fromRotations(0.01);//0.008;
    public static final Rotation2d PIVOT_EXTENDED_TARGET = Rotation2d.fromRotations(0.0);
    
    public static final Rotation2d SETPOINT_CONTROL_TOLERENCE = Rotation2d.fromDegrees(8.0);

    public static final double PIVOT_AGAINST_AMP_ANGLE_THRESHOLD = 0.31; // temporary


  }

  public static class ShooterConstants {
    public static final int LEFT_FLYWHEEL_ID = 30;
    public static final int RIGHT_FLYWHEEL_ID = 31;
    public static final int SHOOTER_PIVOT_ID = 32;
    public static final int HOPPER_ID = 33; 
    public static final int PIVOT_ENCODER_ID = 34;

    public static final double PITCH_RACK_FULL_ROT_TEETH = 440;
    public static final double PITCH_PINION_TEETH = 20;
    public static final double PITCH_PLANITARY_RATIO = 9.0;
    public static final double PITCH_RATIO = (PITCH_RACK_FULL_ROT_TEETH/PITCH_PINION_TEETH) * PITCH_PLANITARY_RATIO;
    public static final double PITCH_KP = 7.5;
    public static final double PITCH_KI = 0.0;
    public static final double PITCH_KD = 0.8;
    public static final double PITCH_KS = 0.025;
    public static final double PITCH_KA = 0.18;
    public static final double PITCH_KV = 17.25;
    public static final double PITCH_KG = 0.43;
    public static final double PITCH_ENCODER_OFFSET = -0.40625+0.125;

    public static final double PITCH_MAX_ROT = 0.21;
    public static final double PITCH_MIN_ROT = 0.01;

    public static final Rotation2d STOW_ANGLE = Rotation2d.fromRotations(0.016);

    public static final SmartSpoolConfig SPEAKER_SMART_SPOOL_CONFIG = new SmartSpoolConfig(6.5, 8.0);

    public static final List<Entry<Double, FireingTableValue>> SPEAKER_FIREING_TABLE = Arrays.asList(
      Map.entry(1.307, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(53.0), 95.0/*80.0*/), 0.0)),
      Map.entry(2.249, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(45.5), 95.0/*81.0*/), 0.0)),
      Map.entry(2.604, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(40.5), 95.0/*82.0*/), 0.0)),
      Map.entry(3.099, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(37.5), 95.0/*85.0*/), 0.0)),
      Map.entry(3.275, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(33.5), 95.0/*84.0*/), 0.0)),
      Map.entry(4.071, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(30.0), 95.0/*93.0*/), 0.0)),
      Map.entry(4.505, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(27.5), 95.0/*94.0*/), 0.0)),
      Map.entry(5.067, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(25.8), 95.0), 0.0)),
      Map.entry(6.260, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(23.5), 95.0/*100.0*/), 0.0))
    );

    public static final SmartSpoolConfig NOTE_PASS_SMART_SPOOL_CONFIG = new SmartSpoolConfig(10.0, 12.0);

    public static final List<Entry<Double, FireingTableValue>> PASS_FIREING_TABLE = Arrays.asList(
      Map.entry(1.307, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(53.0), 95.0/*80.0*/), 0.0)),
      Map.entry(2.249, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(45.5), 95.0/*81.0*/), 0.0)),
      Map.entry(2.604, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(40.5), 95.0/*82.0*/), 0.0)),
      Map.entry(3.099, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(37.5), 95.0/*85.0*/), 0.0)),
      Map.entry(3.275, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(33.5), 95.0/*84.0*/), 0.0)),
      Map.entry(4.071, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(30.0), 95.0/*93.0*/), 0.0)),
      Map.entry(4.505, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(27.5), 95.0/*94.0*/), 0.0)),
      Map.entry(5.067, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(25.8), 95.0), 0.0)),
      Map.entry(6.260, new FireingTableValue(new BreakerVector2(Rotation2d.fromDegrees(23.5), 95.0/*100.0*/), 0.0))
    );

    public static final SmartSpoolConfig SPEAKER_MANUAL_SMART_SPOOL_CONFIG = new SmartSpoolConfig(2.0, 3.0);

    public static final double SHOOTER_IDLE = 0.0;//1500.0 / 60.0;
    public static final BreakerVector2 MANUAL_SPEAKER_SHOT_FIREING_VECTOR = new BreakerVector2(Rotation2d.fromRotations(0.145), 90.0);
  }

  public static class VisionConstants {
    public static final String BACK_RIGHT_CAMERA_NAME = "BackRightCam";
    public static final Transform3d BACK_RIGHT_CAMERA_TRANS = new Transform3d(Units.inchesToMeters(-12.728*2), Units.inchesToMeters(-11.09), Units.inchesToMeters(8.722+1.094), new Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(180-(18.0/2))));

    public static final String BACK_LEFT_CAMERA_NAME = "BackLeftCam";
    public static final Transform3d BACK_LEFT_CAMERA_TRANS = new Transform3d(Units.inchesToMeters(-12.728*2), Units.inchesToMeters(11.09), Units.inchesToMeters(8.722+1.094), new Rotation3d(0.0, Math.toRadians(30.0),(2*Math.PI -(2.78+(0.37/2)) )));


    public static final String FRONT_LEFT_CAMERA_NAME = "FrontLeftCam";
    public static final Transform3d FRONT_LEFT_CAMERA_TRANS = new Transform3d(Units.inchesToMeters(3.436), Units.inchesToMeters(10.789), Units.inchesToMeters(17.47+1.094), new Rotation3d(0.0, Math.toRadians(10.5), Math.toRadians(45.0 - 1.4)));


    public static final String FRONT_RIGHT_CAMERA_NAME = "FrontRightCam";
    public static final Transform3d FRONT_RIGHT_CAMERA_TRANS = new Transform3d(Units.inchesToMeters(3.436), Units.inchesToMeters(-10.789), Units.inchesToMeters(17.47+1.094), new Rotation3d(0.0, Math.toRadians(10.5), Math.toRadians(-45.0+4.5)));

    public static final String LIMELIGHT_NAME = "limelight";
    public static final Transform3d LIMELIGHT_TRANS = new Transform3d();

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  }

  public static class FlywheelConstants {
    public static final double MAX_SPEED = BreakerUnits.rotationsPerMinuteToRotationsPerSecond(6380.0);
    public static final double VELOCITY_TOLERENCE = BreakerUnits.rotationsPerMinuteToRotationsPerSecond(5.0);
    public static final double ACCELERATION_TOLERENCE = BreakerUnits.rotationsPerMinuteToRotationsPerSecond(1.0);
    public static final double CONTACT_WHEEL_CIRCUMFRENCE = Units.inchesToMeters(Math.PI*4.0);
    public static final double MOTOR_TO_FLYWHEEL_GEARING = 1.0;
    public static final double METERS_PER_SEC_PER_MOTOR_ROT = (1.0 / MOTOR_TO_FLYWHEEL_GEARING) * CONTACT_WHEEL_CIRCUMFRENCE;

  }

  public static class ClimbConstants {
    public static final double HOMEING_DUTY_CYCLE = -0.3;
    public static final double HOMEING_CURRENT = 20.0;
    public static final double LEVELING_SOFT_STOP_CURRENT = 10.0;
    public static final double LEVELING_CURRENT_LIMIT = 15.0;
    public static final double HOMEING_CURRENT_LIMIT = 30.0;
    public static final double STANDARD_CURRENT_LIMIT = 65.0;
    public static final double EXTENDED_POSITION_ROTATIONS = 0.0;
    public static final double RETRACTED_POSITION_ROTATIONS = 0.0;
  }

  public static class DriveConstants {
    // Drive motor IDs
    public static final int FL_DRIVE_ID = 10;
    public static final int FR_DRIVE_ID = 12;
    public static final int BL_DRIVE_ID = 14;
    public static final int BR_DRIVE_ID = 16;

    //Azimuth motor IDs
    public static final int FL_TURN_ID = 11;
    public static final int FR_TURN_ID = 13;
    public static final int BL_TURN_ID = 15;
    public static final int BR_TURN_ID = 17;

    //Azimuth Encoder IDs
    public static final int FL_ENCODER_ID = 20;
    public static final int FR_ENCODER_ID = 21;
    public static final int BL_ENCODER_ID = 22;
    public static final int BR_ENCODER_ID = 23;

    //Azimuth encoder angle offets
    public static final double FL_ENCODER_OFFSET = -0.223388671875;
    public static final double FR_ENCODER_OFFSET = -0.06591796875;
    public static final double BL_ENCODER_OFFSET = -0.325439453125;
    public static final double BR_ENCODER_OFFSET = 0.412353515625;

    //Module wheel centerpoint locations relative to robot origin (center)
    public static final Translation2d FL_TRANSLATION = new Translation2d(0.314325, 0.314325);
    public static final Translation2d FR_TRANSLATION = new Translation2d(0.314325, -0.314325);
    public static final Translation2d BL_TRANSLATION = new Translation2d(-0.314325, 0.314325);
    public static final Translation2d BR_TRANSLATION = new Translation2d(-0.314325, -0.314325);

    //Module Azimuth PIDF constants
    public static final double MODULE_AZIMUTH_KP = 45.0;//20
    public static final double MODULE_AZIMUTH_KI = 0.0;
    public static final double MODULE_AZIMUTH_KD = 0.35;//0.3
    public static final double MODULE_AZIMUTH_KF = 0.0;
    public static final BreakerSwerveMotorPIDConfig MODULE_ANGLE_PID_CONFIG = new BreakerSwerveMotorPIDConfig(MODULE_AZIMUTH_KP, MODULE_AZIMUTH_KI, MODULE_AZIMUTH_KD, MODULE_AZIMUTH_KF);

    //Module Drive Velocity PIDF constants
    public static final double MODULE_VELOCITY_KP = 0.17 * 12;//65, 0.10810557184750733
    public static final double MODULE_VELOCITY_KI = 0.0;
    public static final double MODULE_VELOCITY_KD = 0.0;
    public static final double MODULE_VELOCITY_KF = 0.0;
    public static final BreakerSwerveMotorPIDConfig MODULE_VELOCITY_PID_CONFIG = new BreakerSwerveMotorPIDConfig(MODULE_VELOCITY_KP, MODULE_VELOCITY_KI, MODULE_VELOCITY_KD, MODULE_VELOCITY_KF);

    //Module Drive Arbitrary FeedForward
    public static final double FF_STATIC_FRICTION_COEFFICIENT = 0.03 * 12;//3.0, 0.3
    public static final double FF_VELOCITY_COEFFICIENT = 0.145 * 12;//0.0, 2.82
    public static final BreakerArbitraryFeedforwardProvider MODULE_VELOCITY_FF = new BreakerArbitraryFeedforwardProvider(FF_STATIC_FRICTION_COEFFICIENT, FF_VELOCITY_COEFFICIENT);

    public static final double MAX_ATTAINABLE_MODULE_WHEEL_SPEED = 5.4;
    public static final double DRIVE_MOTOR_GEAR_RATIO_TO_ONE = 5.76;
    public static final double AZIMUTH_MOTOR_GEAR_RATIO_TO_ONE = 21.4285714286;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
    public static final double MODULE_WHEEL_SPEED_DEADBAND = 0.001;
    public static final double AZIMUTH_MOTOR_SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double DRIVE_MOTOR_SUPPLY_CURRENT_LIMIT = 10.0;
    public static final BreakerSwerveModuleConfig MODULE_CONFIG = new BreakerSwerveModuleConfig(
      DRIVE_MOTOR_GEAR_RATIO_TO_ONE, AZIMUTH_MOTOR_GEAR_RATIO_TO_ONE, 
      WHEEL_DIAMETER, 
      MAX_ATTAINABLE_MODULE_WHEEL_SPEED,  
      AZIMUTH_MOTOR_SUPPLY_CURRENT_LIMIT, DRIVE_MOTOR_SUPPLY_CURRENT_LIMIT,
      MODULE_ANGLE_PID_CONFIG, MODULE_VELOCITY_PID_CONFIG, 
      MODULE_VELOCITY_FF
    );

      //Theta-axis positional PID
      public static final double HEADING_COMPENSATION_PID_KP = 2.8;
      public static final double HEADING_COMPENSATION_PID_KI = 0.0;
      public static final double HEADING_COMPENSATION_PID_KD = 0.0;
      public static final PIDController HEADING_COMPENSATION_PID = new PIDController(HEADING_COMPENSATION_PID_KP, HEADING_COMPENSATION_PID_KI, HEADING_COMPENSATION_PID_KD);

    //heading snap constants
    public static final double HEADING_SNAP_VEL_RAD_PER_SEC = 2*Math.PI;
    public static final double HEADING_SNAP_ACCEL_RAD_PER_SEC_SQ = 10.0;
    public static final double HEADING_SNAP_POSITIONAL_TOLERENCE_RAD = Math.toRadians(2.5);
    public static final double HEADING_SNAP_VELOCITY_TOLERENCE_RAD_PER_SEC = Math.toRadians(9999);
    public static final double HEADING_SNAP_TIMEOUT_SEC = 5.0;
    public static final double HEADING_SNAP_PID_KP = 3.5;
    public static final double HEADING_SNAP_PID_KI = 0.0;
    public static final double HEADING_SNAP_PID_KD = 0.0;
    public static final Constraints HEADING_SNAP_PID_CONSTRAINTS = new Constraints(HEADING_SNAP_VEL_RAD_PER_SEC, HEADING_SNAP_ACCEL_RAD_PER_SEC_SQ);
    public static final ProfiledPIDController HEADING_SNAP_PID = new ProfiledPIDController(HEADING_SNAP_PID_KP,  HEADING_SNAP_PID_KI,  HEADING_SNAP_PID_KD, HEADING_SNAP_PID_CONSTRAINTS);


    //Slow mode constants
    public static final double SLOW_MODE_LINEAR_MULTIPLIER = 0.5;
    public static final double SLOW_MODE_TURN_MULTIPLIER = 0.5;

    //Physical Robot Constants
    public static final double MAX_ANGULAR_VEL = (2.0 * Math.PI) / ((0.4445 * 2.0 * Math.PI) / (MAX_ATTAINABLE_MODULE_WHEEL_SPEED - 0.2)); 
    public static final double MAX_LINEAR_VEL = 5.0;//5.35
    public static final double HEADING_COMPENSATION_ANGULAR_VEL_DEADBAND = 0.001;
    public static final double HEADING_COMPENSATION_MIN_ACTIVE_LINEAR_VEL = 0.05;
    public static final BreakerSwerveDriveConfig DRIVE_BASE_CONFIG = new BreakerSwerveDriveConfig(
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL, MODULE_WHEEL_SPEED_DEADBAND)
    .withHeadingCompensation(HEADING_COMPENSATION_ANGULAR_VEL_DEADBAND, HEADING_COMPENSATION_MIN_ACTIVE_LINEAR_VEL, HEADING_COMPENSATION_PID)
    .withSlowModeMultipliers(SLOW_MODE_LINEAR_MULTIPLIER, SLOW_MODE_TURN_MULTIPLIER);

    public static final BreakerSwerveOdometryConfig ODOMETRY_CONFIG = new BreakerSwerveOdometryConfig(1.0/200.0,  VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9), PoseOrigin.ofGlobal(), new Pose2d(), 5);

    public static final double X_PID_KP = 2.5;//0.000001
    public static final double X_PID_KI = 0.0;
    public static final double X_PID_KD = 0.008;

    //Y-axis positional PID
    public static final double Y_PID_KP = 2.5;
    public static final double Y_PID_KI = 0.0;
    public static final double Y_PID_KD = 0.008;

    //Theta-axis positional PID
    public static final double THETA_PID_KP = 5.5;
    public static final double THETA_PID_KI = 0.0;
    public static final double THETA_PID_KD = 0.0;

    public static final double LINEAR_PID_KP = Math.hypot(X_PID_KP, Y_PID_KP);
    public static final double LINEAR_PID_KI = Math.hypot(X_PID_KP, Y_PID_KP);
    public static final double LINEAR_PID_KD = Math.hypot(X_PID_KP, Y_PID_KP);

    public static final ReplanningConfig AUTO_REPLANNING_CONFIG = new ReplanningConfig(true, false) ;
    public static final BreakerPathplannerStandardSwerveAutoConfig AUTO_CONFIG = new BreakerPathplannerStandardSwerveAutoConfig(new PIDConstants(LINEAR_PID_KP, LINEAR_PID_KI, LINEAR_PID_KD), new PIDConstants(THETA_PID_KP, THETA_PID_KI, THETA_PID_KD), AUTO_REPLANNING_CONFIG, AZIMUTH_MOTOR_GEAR_RATIO_TO_ONE, true);

    public static final ProfiledPIDController SPEAKER_ALLIGN_ANGLE_PID = new ProfiledPIDController(1.4, 0.0, 0.1, new Constraints(1.0, 3.0));

    
    public static final BreakerSwerveVelocityRequest MOVE_TO_POSE_REQUEST = new BreakerSwerveVelocityRequest(new ChassisSpeeds(), SwerveMovementRefrenceFrame.FIELD_RELATIVE_WITHOUT_OFFSET, SlowModeValue.DISABLED, new Translation2d(), 0.02, false, false);
  }

  public static class AutoConstants {
    public static final PathConstraints PATHFIND_TO_AUTOPATH_START_CONSTRAINTS = new PathConstraints(3.5, 6.0, DriveConstants.MAX_ANGULAR_VEL, 25.0);
  }

  public static class AmpBarConstants {
    public static final Rotation2d EXTENDED_ANGLE_THRESHOLD = Rotation2d.fromRotations(0.39);
    public static final Rotation2d RETRACTED_ANGLE_THRESHOLD = Rotation2d.fromRotations(0.1);
    public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(-0.523681640625);
  
    public static final int AMP_BAR_ENCODER_ID = 35;
    public static final int AMP_BAR_MOTOR_ID = 60;
  }
}
