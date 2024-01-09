// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.physics.vector.BreakerVector3;

/** Interface for 3-axis magnitometers with digital compass features  */
public interface BreakerGenericMagnetometer {
   
    /** in microteslas */
   public abstract BreakerVector3 getRawFieldStrengths();

    /** in microteslas */
   public abstract BreakerVector3 getBiasedFieldStrengths();


   /** @return Magnetic field strength in microteslas. */
   public abstract double getCompassFieldStrength();

   /** @return Angular heading of the compass in +-180 degrees. */
   public abstract Rotation2d getCompassHeading();

   /** @return Raw angular heading of the compass in degrees. */
   public abstract double getRawCompassHeading();

}
