// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.vector;

/** Add your docs here. */
public class BreakerAngularVector3 {

    private double magnitudeYaw;
    private double magnitudePitch;
    private double magnitudeRoll;

    public BreakerAngularVector3(double magnatudeYaw, double magnatudePitch, double magnatudeRoll) {
        this.magnitudeYaw = magnatudeYaw;
        this.magnitudePitch = magnatudePitch;
        this.magnitudeRoll = magnatudeRoll;
    }

    public BreakerAngularVector3() {
        this.magnitudeYaw = 0;
        this.magnitudePitch = 0;
        this.magnitudeRoll = 0;
    }

    
    /** 
     * @return Yaw magnitude of vector.
     */
    public double getMagnitudeYaw() {
        return magnitudeYaw;
    }

    
    /** 
     * @return Pitch magnitude of vector.
     */
    public double getMagnitudePitch() {
        return magnitudePitch;
    }

    
    /** 
     * @return Roll magnitude of vector.
     */
    public double getMagnitudeRoll() {
        return magnitudeRoll;
    }
}
