// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
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
  public static class IntakeConstants {
    public static final int ROLLER_MOTOR_ID = 0; // TODO
    public static final int PIVOT_MOTOR_ID = 0; // TODO
  }

  public static class ShooterConstants {
    public static final BreakerInterpolatingTreeMap<Double, BreakerVector2> FIREING_MAP = getFireingMap();

    private static BreakerInterpolatingTreeMap<Double, BreakerVector2> getFireingMap() {
      BreakerInterpolatingTreeMap<Double, BreakerVector2> fm = new BreakerInterpolatingTreeMap<>();
      fm.put(1.0, new BreakerVector2(Rotation2d.fromDegrees(0.0), 0000));
      return fm;
    } 
  }

  public static class VisionConstants {
    public static final String FRONT_CAMERA_NAME = "frontCam";
    public static final Transform3d FRONT_CAMERA_TRANS = new Transform3d();

    public static final String LEFT_CAMERA_NAME = "leftCam";
    public static final Transform3d LEFT_CAMERA_TRANS = new Transform3d();


    public static final String RIGHT_CAMERA_NAME = "rightCam";
    public static final Transform3d RIGHT_CAMERA_TRANS = new Transform3d();


    public static final String BACK_CAMERA_NAME = "backCam";
    public static final Transform3d BACK_CAMERA_TRANS = new Transform3d();

    public static final String LIMELIGHT_NAME = "limelight";
    public static final Transform3d LIMELIGHT_TRANS = new Transform3d();

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  }

  public static class ClimbConstants {
    
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
      public static final double FL_ENCODER_OFFSET = -0.0;
      public static final double FR_ENCODER_OFFSET = -0.238037;
      public static final double BL_ENCODER_OFFSET = 0.11865234375;
      public static final double BR_ENCODER_OFFSET = 0.33837890625;

      //Module wheel centerpoint locations relative to robot origin (center)
      public static final Translation2d FL_TRANSLATION = new Translation2d(0.314325, 0.314325);
      public static final Translation2d FR_TRANSLATION = new Translation2d(0.314325, -0.314325);
      public static final Translation2d BL_TRANSLATION = new Translation2d(-0.314325, 0.314325);
      public static final Translation2d BR_TRANSLATION = new Translation2d(-0.314325, -0.314325);
  }
}
