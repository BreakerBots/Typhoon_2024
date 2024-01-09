// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Add your docs here. */
public class BreakerFlywheelConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kV;
    private double kS;
    private double velocityTolerence;
    private double acclerationTolerence;
    private double flywheelGearRatio;

    public BreakerFlywheelConfig(double kP, double kI, double kD, double kV, double kS, double velocityTolerence, double accelerationTolerence, double flywheelGearRatio) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
        this.velocityTolerence= velocityTolerence;
        this.acclerationTolerence = accelerationTolerence;
        this.flywheelGearRatio = flywheelGearRatio;
    }

    
    /** 
     * @return double
     */
    public double getkD() {
        return kD;
    }
    
    /** 
     * @return double
     */
    public double getkI() {
        return kI;
    }

    
    /** 
     * @return double
     */
    public double getkP() {
        return kP;
    }

    public double getkS() {
        return kS;
    }

    public double getkV() {
        return kV;
    }

    
    /** 
     * @return double
     */
    public double getAcclerationTolerence() {
        return acclerationTolerence;
    }

    
    /** 
     * @return double
     */
    public double getVelocityTolerence() {
        return velocityTolerence;
    }

    
    /** 
     * @return double
     */
    public double getFlywheelGearRatio() {
        return flywheelGearRatio;
    }
}
