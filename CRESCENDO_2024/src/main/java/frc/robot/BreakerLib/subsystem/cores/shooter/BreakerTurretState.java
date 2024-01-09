// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class BreakerTurretState {
    private Rotation2d azimuthAngle, pitchAngle;
    private double flywheelRPM;
    public BreakerTurretState(Rotation2d azimuthAngle, Rotation2d pitchAngle, double flywheelRPM) {
        this.pitchAngle = pitchAngle;
        this.azimuthAngle = azimuthAngle;
        this.flywheelRPM = flywheelRPM;
    }

    public Rotation2d getAzimuthAngle() {
        return azimuthAngle;
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public Rotation2d getPitchAngle() {
        return pitchAngle;
    }
    
    /** 
     * @param robotRot
     * @return BreakerTurretState
     */
    public BreakerTurretState toRobotRelativeState(Rotation3d robotRot) {
        return new BreakerTurretState(azimuthAngle.plus(robotRot.toRotation2d()), pitchAngle.plus(new Rotation2d(robotRot.getY())), flywheelRPM);
    }
}
