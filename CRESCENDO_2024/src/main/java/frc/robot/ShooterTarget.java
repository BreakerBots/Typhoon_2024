// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Drive;

/** Add your docs here. */
public class ShooterTarget {
    public static final ShooterTarget SPEAKER = new ShooterTarget();

    public double getDistance2d() {

    }

    public Translation3d getAimPoint() {

    }

    public FireingSolution getFireingSolution() {
        
    }

    public static class FireingSolution {
        public FireingSolution(Rotation2d pitch, Rotation2d yaw, double flywheelVelocityRPS) {

        }
    }
}
