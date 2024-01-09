// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.projectile;

import javax.print.DocFlavor.READER;

import frc.robot.BreakerLib.util.math.BreakerUnits;

/** A class that represents the properties of a projectile such as a ball. */
public class BreakerProjectile {

    private double mass;
    private double dragCoefficient;
    private double crossSectionalArea;

    /**
     * Creates a BreakerProjectile.
     * 
     * @param mass Projectile mass in kg.
     * @param dragCoefficient Drag coefficient from this equation: https://en.wikipedia.org/wiki/Drag_coefficient.
     * @param crossSectionalArea Area of surface perpendicular to longest axis of shape, in m^2.
     */
    public BreakerProjectile(double mass, double dragCoefficient, double crossSectionalArea) {
        this.mass = mass;
        this.dragCoefficient = dragCoefficient;
        this.crossSectionalArea = crossSectionalArea;
    }
    
    /** @return Cross-sectional area, in m^2. */
    public double getCrossSectionalArea() {
        return crossSectionalArea;
    }

    /** @return Drag coefficient. */
    public double getDragCoefficient() {
        return dragCoefficient;
    }

    /** @return Projectile mass in kg. */
    public double getMass() {
        return mass;
    }

    public double getDragForce(double vel) {
            return 0.5*dragCoefficient*BreakerUnits.AIR_DENSITY_KG_PER_METER_CUBED*crossSectionalArea*(vel*vel);
    }

    public double getDragAccel(double vel) {
        return getDragForce(vel) / mass;
    }


}
