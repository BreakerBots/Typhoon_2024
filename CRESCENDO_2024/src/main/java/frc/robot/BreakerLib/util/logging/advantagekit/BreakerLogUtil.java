// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.logging.advantagekit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class BreakerLogUtil {

    public static double[] formatChassisSpeedsForLog(ChassisSpeeds... value) {
        double[] data = new double[value.length * 3];
        for (int i = 0; i < value.length; i++) {
          data[i * 3] = value[i].vxMetersPerSecond;
          data[i * 3 + 1] = value[i].vyMetersPerSecond;
          data[i * 3 + 2] = value[i].omegaRadiansPerSecond;
        }
        return data;
    }

    public static double[] formatPose2dForLog(Pose2d... value) {
        double[] data = new double[value.length * 3];
        for (int i = 0; i < value.length; i++) {
          data[i * 3] = value[i].getX();
          data[i * 3 + 1] = value[i].getY();
          data[i * 3 + 2] = value[i].getRotation().getRadians();
        }
        return data;
    }

    public static double[] formatPose3dForLog(Pose3d...  value) {
        double[] data = new double[value.length * 7];
        for (int i = 0; i < value.length; i++) {
          data[i * 7] = value[i].getX();
          data[i * 7 + 1] = value[i].getY();
          data[i * 7 + 2] = value[i].getZ();
          data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
          data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
          data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
          data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
        }
        return data;
    }

    public static double[] formatSwerveModuleStateForLog(SwerveModuleState... value) {
      double[] data = new double[value.length * 2];
      for (int i = 0; i < value.length; i++) {
        data[i * 2] = value[i].angle.getRadians();
        data[i * 2 + 1] = value[i].speedMetersPerSecond;
      }
      return data;
    }
}
